//------------------------------------------------------------------------------
// FERNANDO MORAES     - 05/OUT/2025
// IVAN PALADIN JUNIOR - 16/MAR/2026
// ANGELO DAL ZOTTO    - 30/ABR/2026
//------------------------------------------------------------------------------
// Direct-mapped cache controller with external memories.
// - Connects two memory buses:
//   (a) "cache_*" connects to a small SRAM used as the cache data store
//   (b) "a_*", "c_*", "d_*", "e_*" connect to the main memory
// - Core side is an OBI subordinate: a hit is passed to the cache SRAM
// - Memory side is a TileLink 1.9.3 TL-C master, one operation at a time,
//   one 32-bit word per beat, without coherence yet (no channel B, every
//   line is held with Trunk permission):
//     fill:          A AcquireBlock NtoT -> D GrantData beats -> E GrantAck
//     evict:         C ReleaseData TtoN beats -> D ReleaseAck
//     write-through: A PutPartialData -> D AccessAck
//   Outside a fill, d_ready is always high: a response may come in the cycle
//   of its request
// - Miss: fetch from RAM, fill cache, then grant (the request holds until gnt)
// - Write (WRITE_BACK=0): write-through — hits go to cache and memory; gnt when the Put is accepted
// - Write (WRITE_BACK=1): write-back   — hits go to cache only; dirty evict on miss
// - Cache SRAM side is an OBI manager, any latency. Its responses can't be
//   stalled, so evicted words wait for channel C in a RingBuffer that a read
//   is only issued into when it has room. GrantData beats wait for the SRAM
//   in another RingBuffer, as a TileLink beat may be retracted, an OBI request not.
//   The SRAM serves either the core (IDLE, PUT) or the line transfers (FILL,
//   GRANT_ACK, EVICT): the state only changes between the two with no SRAM
//   access in flight, so every SRAM response in IDLE and PUT goes to the core.
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

    /* Cache SRAM interface: OBI manager */
    output logic                        cache_req_o,
    input  logic                        cache_gnt_i,
    output logic [CACHE_WIDTH-1:0]      cache_addr_o,
    output logic                        cache_we_o,
    output logic [3:0]                  cache_be_o,
    output logic [31:0]                 cache_wdata_o,
    input  logic                        cache_rvalid_i,
    input  logic [31:0]                 cache_rdata_i,

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
    logic       hit_access;
    logic       put_access;
    logic       core_access;
    logic       sram_issue;
    logic       drained;
    logic       filling;
    logic       fill_write;
    logic       end_fill;
    logic       filled;
    logic       evicting;
    logic       evict_read;
    logic       evict_credit;
    logic       reads_done;
    logic       release_beat;
    logic       end_evict;
    logic       release_sent;
    logic       acked;
    logic       put_written;
    logic       acquire_sent;
    logic       evict_room;
    logic       evict_almost_full;
    logic       evict_push;
    logic       fill_push;
    logic       fill_room;
    logic       fill_valid;
    logic [31:0] fill_data;
    logic [3:0] inflight;
    logic [3:0] inflight_next;
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

    /* Core accesses: hits in IDLE, except write-through writes, which write the */
    /* SRAM in PUT once their PutPartialData was accepted (the request holds)    */
    assign hit_access   = req_i && !miss && (current_state == IDLE) && !put_hit;
    assign put_access   = req_i && (current_state == PUT) && !put_written;
    assign core_access  = hit_access || put_access;

    /* SRAM accesses in flight. ponytail: 4 bits, widen if the SRAM takes more than 15 */
    assign sram_issue    = cache_req_o && cache_gnt_i;
    assign inflight_next = inflight + 4'(sram_issue) - 4'(cache_rvalid_i);
    assign drained       = (inflight_next == '0);

    /* Fill: GrantData beats wait in fill_buf for the SRAM */
    assign filling      = (current_state == FILL);
    assign fill_push    = filling && d_valid_i;
    assign fill_write   = filling && fill_valid && cache_gnt_i;
    assign end_fill     = fill_write && (beat == word_idx_t'(FILL_WORDS-1));

    /* Evict: a read is issued only if evict_buf has room for it and for the one in */
    /* flight; ReleaseAck may come at any time after the first beat (spec 4.2.2)    */
    assign evicting     = (current_state == EVICT);
    assign evict_credit = (inflight == '0) ? evict_room
                        : (inflight == 4'd1) && !evict_almost_full;
    assign evict_read   = evicting && !reads_done && evict_credit;
    assign evict_push   = evicting && cache_rvalid_i;
    assign release_beat = c_valid_o && c_ready_i;
    assign end_evict    = release_beat && (beat == word_idx_t'(FILL_WORDS-1));

    always_comb begin
        unique case (current_state)
            IDLE:  begin
                if (req_i && miss && drained)
                    next_state = is_dirty ? EVICT : FILL;
                else if (put_hit && a_ready_i)
                    next_state = PUT;
                else
                    next_state = IDLE;
            end
            FILL:      next_state = ((filled || end_fill) && drained) ? GRANT_ACK : FILL;
            GRANT_ACK: next_state = e_ready_i ? IDLE : GRANT_ACK;
            EVICT:     next_state = ((release_sent || end_evict) && (acked || d_valid_i)) ? IDLE : EVICT;
            PUT:       next_state = ((put_written || (put_access && cache_gnt_i)) && (acked || d_valid_i)) ? IDLE : PUT;
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
            inflight <= '0;
        else
            inflight <= inflight_next;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            acquire_sent <= 1'b0;
        else
            acquire_sent <= filling && (acquire_sent || a_ready_i);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            filled <= 1'b0;
        else
            filled <= filling && (filled || end_fill);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reads_done <= 1'b0;
        else
            reads_done <= evicting && (reads_done || (sram_issue && read_beat == word_idx_t'(FILL_WORDS-1)));
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            release_sent <= 1'b0;
        else
            release_sent <= evicting && (release_sent || end_evict);
    end

    /* ReleaseAck or AccessAck, which may come with the first beat of the request */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            acked <= 1'b0;
        else
            acked <= (next_state == EVICT || next_state == PUT) && (acked || d_valid_i);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            put_written <= 1'b0;
        else
            put_written <= (current_state == PUT) && (put_written || (put_access && cache_gnt_i));
    end

    /* Fill: beat written to the SRAM. Evict: beat sent on channel C */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            beat <= '0;
        else if (filling)
            beat <= beat + word_idx_t'(fill_write);
        else if (evicting)
            beat <= beat + word_idx_t'(release_beat);
        else
            beat <= '0;
    end

    /* Evict: beat read from the SRAM */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            read_beat <= '0;
        else if (evicting)
            read_beat <= read_beat + word_idx_t'(sram_issue);
        else
            read_beat <= '0;
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

    /* verilator lint_off PINCONNECTEMPTY */
    RingBuffer #(
        .DATA_SIZE              (32),
        .BUFFER_SIZE            (2 ),
        .FIRST_WORD_FALL_THROUGH(1 ),
        .ALMOST_FULL_THRESHOLD  (1 ),
        .ALMOST_EMPTY_THRESHOLD (0 )
    ) evict_buf (
        .clk_i         (clk              ),
        .rst_ni        (rst_n            ),
        .buf_rst_i     (1'b0             ),
        .rx_i          (evict_push       ),
        .rx_ack_o      (evict_room       ),
        .data_i        (cache_rdata_i    ),
        .tx_o          (c_valid_o        ),
        .tx_ack_i      (c_ready_i        ),
        .data_o        (c_data_o         ),
        .count_o       (                 ),
        .almost_full_o (evict_almost_full),
        .almost_empty_o(                 )
    );

    /* No fall through: the SRAM request must hold even if d_valid is retracted */
    RingBuffer #(
        .DATA_SIZE              (32),
        .BUFFER_SIZE            (2 ),
        .FIRST_WORD_FALL_THROUGH(0 ),
        .ALMOST_FULL_THRESHOLD  (0 ),
        .ALMOST_EMPTY_THRESHOLD (0 )
    ) fill_buf (
        .clk_i         (clk         ),
        .rst_ni        (rst_n       ),
        .buf_rst_i     (1'b0        ),
        .rx_i          (fill_push   ),
        .rx_ack_o      (fill_room   ),
        .data_i        (d_data_i    ),
        .tx_o          (fill_valid  ),
        .tx_ack_i      (fill_write  ),
        .data_o        (fill_data   ),
        .count_o       (            ),
        .almost_full_o (            ),
        .almost_empty_o(            )
    );
    /* verilator lint_on PINCONNECTEMPTY */

////////////////////////////////////////////////////////////////////////////////
//  Cache interface
////////////////////////////////////////////////////////////////////////////////

    always_comb begin
        unique case (current_state)
            FILL: begin
                cache_req_o   = fill_valid;
                cache_addr_o  = {line_idx, beat, 2'b00};
                cache_we_o    = 1'b1;
                cache_be_o    = 4'hF;
                cache_wdata_o = fill_data;
            end
            EVICT: begin
                cache_req_o   = evict_read;
                cache_addr_o  = {line_idx, read_beat, 2'b00};
                cache_we_o    = 1'b0;
                cache_be_o    = 4'hF;
                cache_wdata_o = fill_data;
            end
            default: begin
                cache_req_o   = core_access;
                cache_addr_o  = {line_idx, offset};
                cache_we_o    = we_i;
                cache_be_o    = be_i;
                cache_wdata_o = wdata_i;
            end
        endcase
    end

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

    /* Channel C: ReleaseData of the dirty line, beats from evict_buf */
    assign c_opcode_o  = TL_C_RELEASE_DATA;
    assign c_param_o   = TL_PRUNE_TTON;
    assign c_size_o    = 4'(OFFSET_WIDTH);
    assign c_source_o  = '0;
    assign c_address_o = {evict_tag, line_idx, offset_t'(0)};
    assign c_corrupt_o = 1'b0;

    /* Channel D: GrantData beats while fill_buf has room; acks always */
    /* ponytail: d_denied and d_corrupt are ignored, add error reporting when a slave can deny */
    assign d_ready_o   = filling ? fill_room : 1'b1;

    /* Channel E: GrantAck echoes the sink of the GrantData */
    assign e_valid_o   = (current_state == GRANT_ACK);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            e_sink_o <= '0;
        else if (fill_push && fill_room)
            e_sink_o <= d_sink_i;
    end

////////////////////////////////////////////////////////////////////////////////
//  Core interface
////////////////////////////////////////////////////////////////////////////////

    /**
     * The SRAM serves core accesses directly. The request waits:
     * - During a miss, until the line is filled (and evicted first if dirty)
     * - Write-through: on write hits, until the memory accepts the PutPartialData
    **/
    assign gnt_o    = core_access && cache_gnt_i;
    assign rvalid_o = cache_rvalid_i && (current_state == IDLE || current_state == PUT);
    assign rdata_o  = cache_rdata_i;

endmodule
