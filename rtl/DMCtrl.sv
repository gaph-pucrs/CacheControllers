//------------------------------------------------------------------------------
// FERNANDO MORAES     - 05/OUT/2025
// IVAN PALADIN JUNIOR - 16/MAR/2026
// ANGELO DAL ZOTTO    - 30/ABR/2026
//------------------------------------------------------------------------------
// Direct-mapped cache controller with external memories.
// - Connects two memory buses:
//   (a) "cache_*" connects to a small SRAM used as the cache data store
//   (b) "a_*", "c_*", "d_*", "e_*" connect to the main memory
// - Core side is an OBI subordinate: gnt on a hit, rvalid in the next cycle
// - Memory side is a TileLink 1.9.3 TL-C master, one operation at a time,
//   one 32-bit word per beat, without coherence yet (no channel B, every
//   line is held with Trunk permission):
//     fill:          A AcquireBlock NtoT -> D GrantData beats -> E GrantAck
//     evict:         C ReleaseData TtoN beats -> D ReleaseAck
//     write-through: A PutPartialData -> D AccessAck
//   d_ready is always high: a response may come in the cycle of its request
// - Miss: fetch from RAM, fill cache, then grant (the request holds until gnt)
// - Write (WRITE_BACK=0): write-through — hits go to cache and memory; gnt when the Put is accepted
// - Write (WRITE_BACK=1): write-back   — hits go to cache only; dirty evict on miss
// - The cache SRAM answers in the next cycle and holds its output while not read
//   Address decode : addr = {tag, index, offset}
// * GAPH - Hardware Design Support Group
// * PUCRS - Pontifical Catholic University of Rio Grande do Sul <https://pucrs.br/>
//------------------------------------------------------------------------------

`include "DMPkg.sv"

module DMCtrl
    import DMPkg::*;
#(
    parameter int unsigned ADDR_WIDTH   = 20,
    parameter int unsigned CACHE_WIDTH  = 12,
    parameter int unsigned OFFSET_WIDTH = 6,
    parameter int unsigned SOURCE_WIDTH = 1,
    parameter int unsigned SINK_WIDTH   = 1,
    parameter write_mode_t WMODE        = WRITE_BACK
)
(
    input  logic                        clk,
    input  logic                        rst_n,

    /* Core interface: OBI subordinate */
    input  logic                        req_i,
    output logic                        gnt_o,
    input  logic [ADDR_WIDTH-1:0]       addr_i,
    input  logic                        we_i,
    input  logic [3:0]                  be_i,
    input  logic [31:0]                 wdata_i,
    output logic                        rvalid_o,
    output logic [31:0]                 rdata_o,

    /* Cache SRAM interface */
    output logic                        cache_ce_o,
    output logic [3:0]                  cache_we_o,
    output logic [CACHE_WIDTH -1:0]     cache_addr_o,
    input  logic [31:0]                 cache_data_i,
    output logic [31:0]                 cache_data_o,
    /* @todo: add cache_busy_i if the SRAM has a non-zero latency -- needs memory buffering with bypass */

    /* Main memory interface: TileLink master, channel A */
    output logic                        a_valid_o,
    input  logic                        a_ready_i,
    output logic [2:0]                  a_opcode_o,
    output logic [2:0]                  a_param_o,
    output logic [3:0]                  a_size_o,
    output logic [SOURCE_WIDTH-1:0]     a_source_o,
    output logic [ADDR_WIDTH-1:0]       a_address_o,
    output logic [3:0]                  a_mask_o,
    output logic [31:0]                 a_data_o,
    output logic                        a_corrupt_o,

    /* Channel C */
    output logic                        c_valid_o,
    input  logic                        c_ready_i,
    output logic [2:0]                  c_opcode_o,
    output logic [2:0]                  c_param_o,
    output logic [3:0]                  c_size_o,
    output logic [SOURCE_WIDTH-1:0]     c_source_o,
    output logic [ADDR_WIDTH-1:0]       c_address_o,
    output logic [31:0]                 c_data_o,
    output logic                        c_corrupt_o,

    /* Channel D: one operation at a time, so only valid, sink and data are needed */
    input  logic                        d_valid_i,
    output logic                        d_ready_o,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [2:0]                  d_opcode_i,
    input  logic [1:0]                  d_param_i,
    input  logic [3:0]                  d_size_i,
    input  logic [SOURCE_WIDTH-1:0]     d_source_i,
    input  logic                        d_denied_i,
    input  logic                        d_corrupt_i,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [SINK_WIDTH-1:0]       d_sink_i,
    input  logic [31:0]                 d_data_i,

    /* Channel E */
    output logic                        e_valid_o,
    input  logic                        e_ready_i,
    output logic [SINK_WIDTH-1:0]       e_sink_o
);

////////////////////////////////////////////////////////////////////////////////
//  Parameters and definitions
////////////////////////////////////////////////////////////////////////////////

    localparam int unsigned TAG_WIDTH   = ADDR_WIDTH  - CACHE_WIDTH;
    localparam int unsigned INDEX_WIDTH = CACHE_WIDTH - OFFSET_WIDTH;
    localparam int unsigned FILL_WORDS  = (1 << (OFFSET_WIDTH - 2));
    localparam int unsigned WORD_IDX_W  = $clog2(FILL_WORDS);
    localparam int unsigned NB_BLOCKS   = (1 << INDEX_WIDTH);

    typedef logic [TAG_WIDTH-1:0]    tag_t;
    typedef logic [INDEX_WIDTH-1:0]  line_idx_t;
    typedef logic [OFFSET_WIDTH-1:0] offset_t;
    typedef logic [WORD_IDX_W-1:0]   word_idx_t;

    typedef struct packed {
        logic valid;
        logic dirty;
        tag_t tag;
    } entry_t;

////////////////////////////////////////////////////////////////////////////////
//  Control logic
////////////////////////////////////////////////////////////////////////////////

    logic       miss;
    logic       is_dirty;
    logic       is_write;
    logic       put_hit;
    logic       filling;
    logic       evicting;
    logic       evicting_r;
    logic       acquire_sent;
    logic       release_sent;
    logic       release_acked;
    logic       fill_beat;
    logic       end_fill;
    logic       release_beat;
    logic       end_evict;
    logic       evict_read;
    tag_t       tag;
    tag_t       evict_tag;
    fsm_t       next_state;
    fsm_t       current_state;
    offset_t    offset;
    line_idx_t  line_idx;
    word_idx_t  beat;
    word_idx_t  read_beat;
    entry_t     entries       [NB_BLOCKS];

    /* OBI holds addr_i until gnt, which only comes after the fill: */
    /* fill and evict can use addr_i without registering it         */
    assign tag          = addr_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign miss         = !entries[line_idx].valid || entries[line_idx].tag != tag;
    assign offset       = addr_i[OFFSET_WIDTH-1:0];
    assign line_idx     = addr_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign evict_tag    = entries[line_idx].tag;
    assign is_dirty     = (WMODE == WRITE_BACK) && entries[line_idx].valid && entries[line_idx].dirty;
    assign is_write     = req_i && we_i;
    assign put_hit      = (WMODE == WRITE_THROUGH) && is_write && !miss && (current_state == IDLE);

    /* GrantData beats are written to the cache as they arrive */
    assign filling      = (current_state == FILL);
    assign fill_beat    = filling && d_valid_i;
    assign end_fill     = fill_beat && (beat == word_idx_t'(FILL_WORDS-1));

    /* ReleaseData streams the line: the SRAM word read in a cycle is the beat   */
    /* offered in the next one, and the SRAM holds it while the beat stalls.     */
    /* ReleaseAck may come at any time after the first beat (spec 4.2.2), so the */
    /* eviction ends when both the last beat is sent and the ack has come.       */
    assign evicting     = (current_state == EVICT);
    assign release_beat = c_valid_o && c_ready_i;
    assign end_evict    = release_beat && (beat == word_idx_t'(FILL_WORDS-1));
    assign evict_read   = evicting && (!evicting_r || c_ready_i);
    assign read_beat    = beat + word_idx_t'(release_beat);

    always_comb begin
        unique case (current_state)
            IDLE:  begin
                if (req_i && miss)
                    next_state = is_dirty ? EVICT : FILL;
                else if (put_hit && a_ready_i && !d_valid_i)
                    next_state = PUT;
                else
                    next_state = IDLE;
            end
            FILL:      next_state = end_fill  ? GRANT_ACK : FILL;
            GRANT_ACK: next_state = e_ready_i ? IDLE      : GRANT_ACK;
            EVICT:     next_state = ((release_sent || end_evict) && (release_acked || d_valid_i)) ? IDLE : EVICT;
            PUT:       next_state = d_valid_i ? IDLE : PUT;
            default:   next_state = IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            evicting_r <= 1'b0;
        else
            evicting_r <= evicting;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            acquire_sent <= 1'b0;
        else
            acquire_sent <= filling && (acquire_sent || a_ready_i);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            release_sent <= 1'b0;
        else
            release_sent <= evicting && (release_sent || end_evict);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            release_acked <= 1'b0;
        else
            release_acked <= evicting && (release_acked || d_valid_i);
    end

    /* Beat received by the fill or offered by the eviction */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            beat <= '0;
        else if (filling)
            beat <= beat + word_idx_t'(fill_beat);
        else if (evicting)
            beat <= read_beat;
        else
            beat <= '0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NB_BLOCKS; i++) begin
                entries[i].valid <= 1'b0;
                entries[i].dirty <= 1'b0;
                entries[i].tag   <= '0;
            end
        end
        else begin
            unique case (current_state)
                IDLE:  begin
                    if ((WMODE == WRITE_BACK) && is_write && !miss)
                        entries[line_idx].dirty <= 1'b1;
                end
                FILL:  begin
                    if (end_fill) begin
                        entries[line_idx].valid <= 1'b1;
                        entries[line_idx].dirty <= 1'b0;
                        entries[line_idx].tag   <= tag;
                    end
                end
                EVICT: begin
                    if (end_evict)
                        entries[line_idx].dirty <= 1'b0;
                end
                default: ;
            endcase
        end
    end

////////////////////////////////////////////////////////////////////////////////
//  Cache interface
////////////////////////////////////////////////////////////////////////////////

    /* Access cache on a granted hit, on each fill beat, or to read the next eviction beat */
    assign cache_ce_o   = gnt_o || fill_beat || evict_read;

    /* Write to cache on write hit or during fill; read otherwise */
    always_comb begin
        unique case (current_state)
            IDLE:    cache_we_o = we_i ? be_i : 4'h0;
            FILL:    cache_we_o = 4'hF;
            default: cache_we_o = 4'h0;
        endcase
    end

    /* Cache address on read hit or write from input, otherwise from fill/evict beat */
    always_comb begin
        unique case (current_state)
            IDLE:    cache_addr_o = {line_idx, offset};
            FILL:    cache_addr_o = {line_idx, beat, 2'b00};
            default: cache_addr_o = {line_idx, read_beat, 2'b00};
        endcase
    end

    /* During fill, use data from main memory, otherwise from input */
    assign cache_data_o = (current_state == IDLE) ? wdata_i : d_data_i;

////////////////////////////////////////////////////////////////////////////////
//  Memory interface
////////////////////////////////////////////////////////////////////////////////

    /* Channel A: AcquireBlock of the missing line, or PutPartialData of a write-through hit */
    assign a_valid_o   = put_hit || (filling && !acquire_sent);
    assign a_opcode_o  = filling ? TL_A_ACQUIRE_BLOCK : TL_A_PUT_PARTIAL_DATA;
    assign a_param_o   = filling ? TL_GROW_NTOT       : 3'd0;
    assign a_size_o    = filling ? 4'(OFFSET_WIDTH)   : 4'd2;
    assign a_source_o  = '0;
    assign a_address_o = filling ? {tag, line_idx, offset_t'(0)} : {addr_i[ADDR_WIDTH-1:2], 2'b00};
    assign a_mask_o    = filling ? 4'hF : be_i;
    assign a_data_o    = wdata_i;
    assign a_corrupt_o = 1'b0;

    /* Channel C: ReleaseData of the dirty line */
    assign c_valid_o   = evicting && evicting_r && !release_sent;
    assign c_opcode_o  = TL_C_RELEASE_DATA;
    assign c_param_o   = TL_PRUNE_TTON;
    assign c_size_o    = 4'(OFFSET_WIDTH);
    assign c_source_o  = '0;
    assign c_address_o = {evict_tag, line_idx, offset_t'(0)};
    assign c_data_o    = cache_data_i;
    assign c_corrupt_o = 1'b0;

    /* Channel D: always accepted (GrantData, ReleaseAck or AccessAck) */
    /* ponytail: d_denied and d_corrupt are ignored, add error reporting when a slave can deny */
    assign d_ready_o   = 1'b1;

    /* Channel E: GrantAck echoes the sink of the GrantData */
    assign e_valid_o   = (current_state == GRANT_ACK);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            e_sink_o <= '0;
        else if (fill_beat)
            e_sink_o <= d_sink_i;
    end

////////////////////////////////////////////////////////////////////////////////
//  Core interface
////////////////////////////////////////////////////////////////////////////////

    /**
     * Grant only hits in IDLE. The request waits:
     * - During a miss, until the line is filled (and evicted first if dirty)
     * - Write-through: on write hits, until the memory accepts the PutPartialData
    **/
    assign gnt_o = req_i && !miss && (current_state == IDLE)
                && !(put_hit && !a_ready_i);

    /* The cache SRAM answers the granted access in the next cycle */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rvalid_o <= 1'b0;
        else
            rvalid_o <= gnt_o;
    end

    assign rdata_o = cache_data_i;

endmodule
