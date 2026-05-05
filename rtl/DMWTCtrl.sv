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
// - Write: write-through 
// - Single Port Memory access: Through a controller for instruction and data usage.
//   Address decode : addr = {tag, index, offset} 
// * GAPH - Hardware Design Support Group
// * PUCRS - Pontifical Catholic University of Rio Grande do Sul <https://pucrs.br/>
//------------------------------------------------------------------------------

module DMWTCtrl #(
    parameter int unsigned ADDR_WIDTH   = 20,
    parameter int unsigned CACHE_WIDTH  = 12,
    parameter int unsigned OFFSET_WIDTH = 6
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

    typedef enum logic [1:0] {
        MEM_IDLE  = 2'b01,
        MEM_FETCH = 2'b10
    } mem_fsm_t;

    typedef enum logic [1:0] {
        CACHE_IDLE = 2'b01,
        CACHE_FILL = 2'b10
    } cache_fsm_t;
    
    typedef struct packed {
        logic valid;
        tag_t tag;
    } entry_t;

////////////////////////////////////////////////////////////////////////////////
//  Common signals
////////////////////////////////////////////////////////////////////////////////

    logic      mem_valid;
    tag_t      tag_r;
    line_idx_t line_idx_r;

////////////////////////////////////////////////////////////////////////////////
//  Cache control
////////////////////////////////////////////////////////////////////////////////

    logic       miss;
    logic       is_write;
    logic       read_miss;
    logic       last_write;
    tag_t       tag;
    offset_t    offset;
    line_idx_t  line_idx;
    word_idx_t  fill_idx;
    cache_fsm_t cache_cs;
    cache_fsm_t cache_ns;
    entry_t     entries   [NB_BLOCKS];

    assign tag        = address_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign miss       = !entries[line_idx].valid || entries[line_idx].tag != tag;
    assign offset     = address_i[OFFSET_WIDTH-1:0];
    assign line_idx   = address_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign is_write   = ce_i && (we_i != '0);
    assign read_miss  = ce_i && (we_i == '0) && miss;
    assign last_write = mem_valid && (fill_idx == word_idx_t'(FILL_WORDS-1));

    always_comb begin
        unique case (cache_cs)
            CACHE_IDLE: cache_ns = read_miss  ? CACHE_FILL : CACHE_IDLE;
            CACHE_FILL: cache_ns = last_write ? CACHE_IDLE : CACHE_FILL;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cache_cs <= CACHE_IDLE;
        else
            cache_cs <= cache_ns;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fill_idx <= '0;
        end 
        else begin
            unique case (cache_cs)
                CACHE_IDLE: fill_idx <= '0;
                CACHE_FILL: fill_idx <= mem_valid ? fill_idx + 1'b1 : fill_idx;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin  
        if (!rst_n) begin
            for (int i = 0; i < NB_BLOCKS; i++) begin
                entries[i].valid <= 1'b0;
                entries[i].tag   <= '0;
            end
        end 
        else begin
            if ((cache_cs == CACHE_FILL) && last_write) begin
                entries[line_idx_r].valid <= 1'b1;
                entries[line_idx_r].tag   <= tag_r;
            end
        end
    end

    /* Access cache on hit or during fill */
    assign cache_ce_o   = ((cache_cs == CACHE_FILL) && mem_valid) || (ce_i && !miss);

    /* Write to cache on write hit or during fill */
    assign cache_we_o   = (cache_cs == CACHE_FILL) ? 4'hF : we_i;

    /* Cache address on read hit or write from input, otherwise from fill */ 
    assign cache_addr_o = (cache_cs == CACHE_FILL) ? {line_idx_r, fill_idx, 2'b00} : {line_idx, offset};

    /* During fill, use data from main memory, otherwise from input */
    assign cache_data_o = (cache_cs == CACHE_FILL) ? mem_data_i : data_i;

////////////////////////////////////////////////////////////////////////////////
//  Memory control
////////////////////////////////////////////////////////////////////////////////

    mem_fsm_t  mem_cs;
    mem_fsm_t  mem_ns;
    word_idx_t fetch_idx;

    /* Address can change even if the cache is busy                          */
    /* Miss control is made with address_i, but memory access with address_r */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [ADDR_WIDTH-1:0] address_r;
    /* verilator lint_on UNUSEDSIGNAL */

    assign tag_r      = address_r[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign line_idx_r = address_r[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];

    always_comb begin
        unique case (mem_cs)
            MEM_IDLE:  mem_ns = read_miss  ? MEM_FETCH : MEM_IDLE;
            MEM_FETCH: mem_ns = last_write ? MEM_IDLE  : MEM_FETCH;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mem_cs <= MEM_IDLE;
        else
            mem_cs <= mem_ns;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_valid <= 1'b0;
        end
        else begin
            unique case (mem_cs)
                MEM_IDLE:  mem_valid <= 1'b0;
                MEM_FETCH: mem_valid <= !mem_busy_i;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            address_r <= '0;
        end 
        else begin
            if (read_miss && (mem_cs == MEM_IDLE))
                address_r <= address_i;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fetch_idx <= '0;
        end 
        else begin
            unique case (mem_cs)
                MEM_IDLE:  fetch_idx <= '0;
                MEM_FETCH: fetch_idx <= mem_busy_i ? fetch_idx : fetch_idx + 1'b1;
            endcase
        end
    end

    /* Enable main memory on write through or during fill */
    assign mem_ce_o   = (mem_cs == MEM_FETCH) || is_write;

    /* Write to main memory only on write through. Force read during fill */
    assign mem_we_o   = (mem_cs == MEM_FETCH) ? 4'h0 : we_i;

    /* Main memory address on write through comes from input, otherwise from fill */
    assign mem_addr_o = (mem_cs == MEM_FETCH) ? {tag_r, line_idx_r, fetch_idx, 2'b00} : address_i;

    /* Data to be written to main memory always comes from write through */
    assign mem_data_o = data_i;

////////////////////////////////////////////////////////////////////////////////
//  Core interface
////////////////////////////////////////////////////////////////////////////////

    /* Only read from cache */
    assign data_o = cache_data_i;

    /**
     * Make the core wait during the following conditions:
     * - During a write through, wait the memory to answer
     * - During a read miss
     * - During a cache fill, even if the miss is satisfied due to an input address change
    **/
    assign busy_o = (cache_cs == CACHE_FILL) || read_miss || (is_write && mem_busy_i);

endmodule
