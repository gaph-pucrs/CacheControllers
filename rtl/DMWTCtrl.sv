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
    
    typedef struct packed {
        logic valid;
        tag_t tag;
    } entry_t;

////////////////////////////////////////////////////////////////////////////////
//  Cache control
////////////////////////////////////////////////////////////////////////////////

    entry_t entries [NB_BLOCKS];

    tag_t tag;
    assign tag = address_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];

    line_idx_t line_idx;
    assign line_idx = address_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];

    offset_t offset;
    assign offset = address_i[OFFSET_WIDTH-1:0];

    logic miss;
    assign miss = !entries[line_idx].valid || entries[line_idx].tag != tag;

    logic read_miss;
    assign read_miss = (we_i == '0) && miss;

    logic filling;
    
    logic mem_valid;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mem_valid <= 1'b0;
        else
            mem_valid <= filling && !mem_busy_i;
    end

    logic last_write;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filling <= 1'b0;
        end
        else begin
            if (!filling)
                filling <= ce_i && read_miss;
            else
                filling <= !(mem_valid && last_write);
        end
    end

    /* Address can change even if the cache is busy                          */
    /* Miss control is made with address_i, but memory access with address_r */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [ADDR_WIDTH-1:0] address_r;
    /* verilator lint_on UNUSEDSIGNAL */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            address_r <= '0;
        end 
        else begin
            if (ce_i && read_miss && !filling)
                address_r <= address_i;
        end
    end

    line_idx_t line_idx_r;
    assign line_idx_r = address_r[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];

    tag_t tag_r;
    assign tag_r = address_r[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];

    word_idx_t fill_idx;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fill_idx <= '0;
        end 
        else begin
            if (!filling)
                fill_idx <= '0;
            else if (mem_valid)
                fill_idx <= fill_idx + 1'b1;
        end
    end

    assign last_write = (fill_idx == word_idx_t'(FILL_WORDS-1));

    word_idx_t fetch_idx;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fetch_idx <= '0;
        end 
        else begin
            if (!filling)
                fetch_idx <= '0;
            else if (!mem_busy_i)
                fetch_idx <= fetch_idx + 1'b1;
        end
    end

    always_ff @(posedge clk or posedge rst_n) begin  
        if (!rst_n) begin
            for (int i = 0; i < NB_BLOCKS; i++) begin
                entries[i].valid <= 1'b0;
                entries[i].tag   <= '0;
            end
        end 
        else begin
            if (filling && last_write) begin
                entries[line_idx_r].valid <= 1'b1;
                entries[line_idx_r].tag   <= tag_r;
            end
        end
    end

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
    assign busy_o = filling || read_miss || (mem_busy_i && (we_i != '0));

////////////////////////////////////////////////////////////////////////////////
//  Cache memory interface
////////////////////////////////////////////////////////////////////////////////

    /* Access cache on hit or during fill */
    assign cache_ce_o = (filling && mem_valid) || (ce_i && !miss);

    /* Write to cache on write hit or during fill */
    assign cache_we_o = filling ? 4'hF : we_i;

    /* Cache address on read hit or write from input, otherwise from fill */ 
    assign cache_addr_o = filling ? {line_idx_r, fill_idx, 2'b00} : {line_idx, offset};

    /* During fill, use data from main memory, otherwise from input */
    assign cache_data_o = filling ? mem_data_i : data_i;

////////////////////////////////////////////////////////////////////////////////
//  Main memory interface
////////////////////////////////////////////////////////////////////////////////

    /* Enable main memory on write through or during fill */
    assign mem_ce_o   = filling || (ce_i && (we_i != '0));

    /* Write to main memory only on write through. Force read during fill */
    assign mem_we_o   = filling ? 4'h0 : we_i;

    /* Main memory address on write through comes from input, otherwise from fill */
    assign mem_addr_o = filling ? {tag_r, line_idx_r, fetch_idx, 2'b00} : address_i;

    /* Data to be written to main memory always comes from write through */
    assign mem_data_o = data_i;

endmodule
