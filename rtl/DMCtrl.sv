//------------------------------------------------------------------------------
// FERNANDO MORAES     - 05/OUT/2025
// IVAN PALADIN JUNIOR - 16/MAR/2026
// ANGELO DAL ZOTTO    - 30/ABR/2026
//------------------------------------------------------------------------------
// Direct-mapped cache controller with external memories.
// - Connects two memory buses:
//   (a) "cache_*" connects to a small SRAM used as the cache data store
//   (b) "mem_*"   connects to the main memory
// - Read miss: fetch from RAM, fill cache, then answer
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

    /* Core interface */
    input  logic                        ce_i,
    input  logic [3:0]                  we_i,
    input  logic [ADDR_WIDTH-1:0]       address_i,
    input  logic [31:0]                 data_i,
    output logic [31:0]                 data_o,
    output logic                        busy_o,

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

    typedef enum logic [2:0] {
        IDLE  = 3'b001,
        FILL  = 3'b010,
        EVICT = 3'b100
    } fsm_t;

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
    tag_t       tag_r;
    tag_t       evict_tag_r;
    fsm_t       next_state;
    fsm_t       current_state;
    offset_t    offset;
    line_idx_t  line_idx;
    line_idx_t  line_idx_r;
    word_idx_t  mem_idx;
    word_idx_t  cache_idx;
    entry_t     entries       [NB_BLOCKS];

    /* Address can change even if the cache is busy                          */
    /* Miss control is made with address_i, but memory access with address_r */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [ADDR_WIDTH-1:0] address_r;
    /* verilator lint_on UNUSEDSIGNAL */

    assign tag         = address_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign tag_r       = address_r[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign miss        = !entries[line_idx].valid || entries[line_idx].tag != tag;
    assign offset      = address_i[OFFSET_WIDTH-1:0];
    assign line_idx    = address_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign line_idx_r  = address_r[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign evict_tag_r = entries[line_idx_r].tag;
    assign is_dirty    = (WMODE == WRITE_BACK) && entries[line_idx].valid && entries[line_idx].dirty;
    assign is_write    = ce_i && (we_i != '0);
    assign end_fill    = mem_valid && (cache_idx == word_idx_t'(FILL_WORDS-1));
    assign end_evict   = cache_valid && !mem_busy_i && (mem_idx == word_idx_t'(FILL_WORDS-1));

    always_comb begin
        unique case (current_state)
            IDLE:  begin
                if (ce_i && miss)
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
                        entries[line_idx_r].valid <= 1'b1;
                        entries[line_idx_r].dirty <= 1'b0;
                        entries[line_idx_r].tag   <= tag_r;
                    end
                end
                EVICT: begin
                    if (end_evict)
                        entries[line_idx_r].dirty <= 1'b0;
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            address_r <= '0;
        else if (ce_i && miss && (current_state == IDLE))
            address_r <= address_i;
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

    /* Access cache during fill, evict or on hit */
    assign cache_ce_o   = (ce_i && !miss && current_state == IDLE) || (current_state == EVICT) || ((current_state == FILL) && mem_valid);

    /* Write to cache on write hit or during fill; force read during evict */
    always_comb begin
        unique case (current_state)
            IDLE:  cache_we_o   = we_i;
            FILL:  cache_we_o   = 4'hF;
            EVICT: cache_we_o   = 4'h0;
        endcase
    end

    /* Cache address on read hit or write from input, otherwise from fill/evict */
    assign cache_addr_o = (current_state == IDLE) ? {line_idx, offset} : {line_idx_r, cache_idx, 2'b00};

    /* During fill, use data from main memory, otherwise from input */
    assign cache_data_o = (current_state == IDLE) ? data_i : mem_data_i;

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
            mem_we_o = we_i;
        else if (current_state == EVICT)
            mem_we_o = 4'hF;
        else
            mem_we_o = 4'h0;
    end

    /* Fill/write-through use the incoming address; evict uses the dirty line's tag */
    assign mem_addr_o = (current_state == EVICT)
                      ? {evict_tag_r, line_idx_r, mem_idx, 2'b00}
                      : ((WMODE == WRITE_THROUGH) && current_state == IDLE)
                        ? address_i
                        : {tag_r, line_idx_r, mem_idx, 2'b00};

    /* Write-through sends CPU data on hits; evict sends cache data */
    assign mem_data_o = ((WMODE == WRITE_THROUGH) && current_state == IDLE) ? data_i : cache_data_i;

////////////////////////////////////////////////////////////////////////////////
//  Core interface
////////////////////////////////////////////////////////////////////////////////

    /* Only read from cache */
    assign data_o = cache_data_i;

    /**
     * Make the core wait during the following conditions:
     * - During a miss
     * - During a cache fill, even if the miss is satisfied due to an input address change
     * - During a cache eviction, even if the miss is satisfied due to an input address change
     * - Write-through: stall on write hits if memory is busy
    **/
    assign busy_o = miss || (current_state != IDLE)
                 || ((WMODE == WRITE_THROUGH) && is_write && !miss && mem_busy_i);

endmodule
