//------------------------------------------------------------------------------
// FERNANDO MORAES     - 05/OUT/2025
// IVAN PALADIN JUNIOR - 16/MAR/2026
// ANGELO DAL ZOTTO    - 30/ABR/2026
//------------------------------------------------------------------------------
// Direct-mapped cache controller with external memories.
// - Connects two memory buses:
//   (a) "cache_*" connects to a small SRAM used as the cache data store
//   (b) "mem_*"   connects to the main memory
// - Core side is an OBI subordinate: gnt on a hit, rvalid in the next cycle
// - Miss: fetch from RAM, fill cache, then grant (the request holds until gnt)
// - Write (WRITE_BACK=0): write-through — hits go to cache and memory immediately
// - Write (WRITE_BACK=1): write-back   — hits go to cache only; dirty evict on miss
// - Single Port Memory access: Through a controller for instruction and data usage.
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

    /* Main memory interface */
    output logic                        mem_ce_o,
    output logic [3:0]                  mem_we_o,
    output logic [ADDR_WIDTH-1:0]       mem_addr_o,
    input  logic [31:0]                 mem_data_i,
    output logic [31:0]                 mem_data_o,
    input  logic                        mem_busy_i
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
    logic       end_fill;
    logic       end_evict;
    logic       mem_valid;
    logic       cache_valid;
    tag_t       tag;
    tag_t       evict_tag;
    fsm_t       next_state;
    fsm_t       current_state;
    offset_t    offset;
    line_idx_t  line_idx;
    word_idx_t  mem_idx;
    word_idx_t  cache_idx;
    entry_t     entries       [NB_BLOCKS];

    /* OBI holds addr_i until gnt, which only comes after the fill: */
    /* fill and evict can use addr_i without registering it         */
    assign tag         = addr_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign miss        = !entries[line_idx].valid || entries[line_idx].tag != tag;
    assign offset      = addr_i[OFFSET_WIDTH-1:0];
    assign line_idx    = addr_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign evict_tag   = entries[line_idx].tag;
    assign is_dirty    = (WMODE == WRITE_BACK) && entries[line_idx].valid && entries[line_idx].dirty;
    assign is_write    = req_i && we_i;
    assign end_fill    = mem_valid && (cache_idx == word_idx_t'(FILL_WORDS-1));
    assign end_evict   = cache_valid && !mem_busy_i && (mem_idx == word_idx_t'(FILL_WORDS-1));

    always_comb begin
        unique case (current_state)
            IDLE:  begin
                if (req_i && miss)
                    next_state = is_dirty ? EVICT : FILL;
                else
                    next_state = IDLE;
            end
            FILL:  next_state = end_fill  ? IDLE : FILL;
            EVICT: next_state = end_evict ? IDLE : EVICT;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cache_idx <= '0;
        end
        else begin
            unique case (current_state)
                IDLE:  cache_idx <= '0;
                FILL:  cache_idx <=                 mem_valid  ? cache_idx + 1'b1 : cache_idx;
                EVICT: cache_idx <= cache_valid && !mem_busy_i ? cache_idx + 1'b1 : cache_idx;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_idx <= '0;
        end
        else begin
            unique case (current_state)
                IDLE:  mem_idx <= '0;
                FILL:  mem_idx <=                !mem_busy_i ? mem_idx + 1'b1 : mem_idx;
                EVICT: mem_idx <= cache_valid && !mem_busy_i ? mem_idx + 1'b1 : mem_idx;
            endcase
        end
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
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_valid <= 1'b0;
        end
        else begin
            unique case (current_state)
                FILL:    mem_valid <= !mem_busy_i;
                default: mem_valid <= 1'b0;
            endcase
        end
    end

    /* If !mem_busy_i, the current cache read is consumed, and thus the read index will be increased */
    /* This invalidates the next data, requiring an additional cycle to read from cache before writing to memory */
    /* This avoids buffering the cache reads in case the main memory is busy during eviction */
    /* But this is very inneficient */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cache_valid <= 1'b0;
        end
        else begin
            unique case (current_state)
                EVICT:   cache_valid <= !(cache_valid && !mem_busy_i);
                default: cache_valid <= 1'b0;
            endcase
        end
    end

////////////////////////////////////////////////////////////////////////////////
//  Cache interface
////////////////////////////////////////////////////////////////////////////////

    /* Access cache during fill, evict or on a granted hit */
    assign cache_ce_o   = gnt_o || (current_state == EVICT) || ((current_state == FILL) && mem_valid);

    /* Write to cache on write hit or during fill; force read during evict */
    always_comb begin
        unique case (current_state)
            IDLE:  cache_we_o   = we_i ? be_i : 4'h0;
            FILL:  cache_we_o   = 4'hF;
            EVICT: cache_we_o   = 4'h0;
        endcase
    end

    /* Cache address on read hit or write from input, otherwise from fill/evict */
    assign cache_addr_o = (current_state == IDLE) ? {line_idx, offset} : {line_idx, cache_idx, 2'b00};

    /* During fill, use data from main memory, otherwise from input */
    assign cache_data_o = (current_state == IDLE) ? wdata_i : mem_data_i;

////////////////////////////////////////////////////////////////////////////////
//  Memory interface
////////////////////////////////////////////////////////////////////////////////

    /* Write-through: also enable memory on write hits in IDLE */
    /* Write-back:    only enable memory during fill or eviction */
    assign mem_ce_o = (current_state == FILL)
                   || ((current_state == EVICT) && cache_valid)
                   || ((WMODE == WRITE_THROUGH) && is_write && !miss && current_state == IDLE);

    /* Write-through writes on hits; write-back writes only on eviction */
    always_comb begin
        if ((WMODE == WRITE_THROUGH) && is_write && !miss && current_state == IDLE)
            mem_we_o = be_i;
        else if (current_state == EVICT)
            mem_we_o = 4'hF;
        else
            mem_we_o = 4'h0;
    end

    /* Fill/write-through use the incoming address; evict uses the dirty line's tag */
    assign mem_addr_o = (current_state == EVICT)
                      ? {evict_tag, line_idx, mem_idx, 2'b00}
                      : ((WMODE == WRITE_THROUGH) && current_state == IDLE)
                        ? addr_i
                        : {tag, line_idx, mem_idx, 2'b00};

    /* Write-through sends CPU data on hits; evict sends cache data */
    assign mem_data_o = ((WMODE == WRITE_THROUGH) && current_state == IDLE) ? wdata_i : cache_data_i;

////////////////////////////////////////////////////////////////////////////////
//  Core interface
////////////////////////////////////////////////////////////////////////////////

    /**
     * Grant only hits in IDLE. The request waits:
     * - During a miss, until the line is filled (and evicted first if dirty)
     * - Write-through: on write hits while the memory is busy
    **/
    assign gnt_o = req_i && !miss && (current_state == IDLE)
                && !((WMODE == WRITE_THROUGH) && we_i && mem_busy_i);

    /* The cache SRAM answers the granted access in the next cycle */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rvalid_o <= 1'b0;
        else
            rvalid_o <= gnt_o;
    end

    assign rdata_o = cache_data_i;

endmodule
