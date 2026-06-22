//------------------------------------------------------------------------------
// FERNANDO MORAES     - 05/OUT/2025
// IVAN PALADIN JUNIOR - 16/MAR/2026
// ANGELO DAL ZOTTO    - 30/ABR/2026
// LUCAS MOTTA DAMO    - 21/JUN/2026
//------------------------------------------------------------------------------
// Direct-mapped cache controller with external memories.
// - Connects two memory buses:
//   (a) "cache_*" connects to a small SRAM used as the cache data store
//   (b) "mem_*"   connects to the main memory
// - Read miss : evict dirty victim (if any), fetch from RAM, fill, then answer
// - Write     : write-back + write-allocate
//     * Hit  : write the word in the cache and mark the line dirty
//     * Miss : evict dirty victim (if any), fetch the line, then write the word
// - Single Port Memory access: Through a controller for instruction and data usage.
//   Address decode : addr = {tag, index, offset}
//
// - Parameterizable data widths (so e.g. the vector unit can later read/write a
//   whole cache line in a single core access):
//     * DATA_WIDTH : core <-> cache-controller (and cache SRAM) word width
//     * MEM_WIDTH  : cache-controller <-> main-memory transfer (beat) width
//   A line is moved between the cache SRAM and main memory in BEATS_PER_LINE
//   beats of MEM_WIDTH bits. When DATA_WIDTH > MEM_WIDTH each cache word spans
//   BEATS_PER_WORD beats. Constraint: DATA_WIDTH is an integer multiple of
//   MEM_WIDTH (DATA_WIDTH == MEM_WIDTH for the scalar default of 32/32).
// * GAPH - Hardware Design Support Group
// * PUCRS - Pontifical Catholic University of Rio Grande do Sul <https://pucrs.br/>
//------------------------------------------------------------------------------

module DMWBCtrl #(
    parameter int unsigned ADDR_WIDTH   = 20,
    parameter int unsigned CACHE_WIDTH  = 12,
    parameter int unsigned OFFSET_WIDTH = 6,
    parameter int unsigned DATA_WIDTH   = 32,
    parameter int unsigned MEM_WIDTH    = 32
)
(
    input  logic                        clk,
    input  logic                        rst_n,

    /* Core interface */
    input  logic                        ce_i,
    input  logic [DATA_WIDTH/8-1:0]     we_i,
    input  logic [ADDR_WIDTH-1:0]       address_i,
    input  logic [DATA_WIDTH-1:0]       data_i,
    output logic [DATA_WIDTH-1:0]       data_o,
    output logic                        busy_o,

    /* Cache SRAM interface (DATA_WIDTH wide data store) */
    output logic                        cache_ce_o,
    output logic [DATA_WIDTH/8-1:0]     cache_we_o,
    output logic [CACHE_WIDTH -1:0]     cache_addr_o,
    input  logic [DATA_WIDTH-1:0]       cache_data_i,
    output logic [DATA_WIDTH-1:0]       cache_data_o,
    /* @todo: add cache_busy_i if the SRAM has a non-zero latency -- needs memory buffering with bypass */

    /* Main memory interface (MEM_WIDTH wide transfers) */
    output logic                        mem_ce_o,
    output logic [MEM_WIDTH/8-1:0]      mem_we_o,
    output logic [ADDR_WIDTH-1:0]       mem_addr_o,
    input  logic [MEM_WIDTH-1:0]        mem_data_i,
    output logic [MEM_WIDTH-1:0]        mem_data_o,
    input  logic                        mem_busy_i
);

////////////////////////////////////////////////////////////////////////////////
//  Parameters and definitions
////////////////////////////////////////////////////////////////////////////////

    localparam int unsigned TAG_WIDTH   = ADDR_WIDTH  - CACHE_WIDTH;
    localparam int unsigned INDEX_WIDTH = CACHE_WIDTH - OFFSET_WIDTH;
    localparam int unsigned NB_BLOCKS   = (1 << INDEX_WIDTH);
    localparam int unsigned LINE_BYTES  = (1 << OFFSET_WIDTH);

    /* Bus geometry */
    localparam int unsigned DATA_BYTES     = DATA_WIDTH / 8;
    localparam int unsigned MEM_BYTES      = MEM_WIDTH  / 8;
    localparam int unsigned MEM_BYTE_W     = (MEM_BYTES  > 1) ? $clog2(MEM_BYTES)  : 0;
    /* A whole line is transferred to/from main memory in BEATS_PER_LINE beats.  */
    /* When the cache word is wider than a beat, BEATS_PER_WORD beats build it.   */
    localparam int unsigned BEATS_PER_LINE = LINE_BYTES / MEM_BYTES;
    localparam int unsigned BEAT_CNT_W     = (BEATS_PER_LINE > 1) ? $clog2(BEATS_PER_LINE) : 1;
    /* Index widths for selecting the beat's lane inside a (wider) cache word */
    localparam int unsigned LANE_BYTE_W    = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 1;
    localparam int unsigned LANE_BIT_W     = (DATA_WIDTH > 1) ? $clog2(DATA_WIDTH) : 1;

    typedef logic [TAG_WIDTH-1:0]    tag_t;
    typedef logic [INDEX_WIDTH-1:0]  line_idx_t;
    typedef logic [OFFSET_WIDTH-1:0] offset_t;
    typedef logic [BEAT_CNT_W-1:0]   beat_idx_t;

    typedef struct packed {
        logic valid;
        logic dirty;
        tag_t tag;
    } entry_t;

    /* A write-back miss is served as a two-phase burst (evict then fill).      */
    /* The cache SRAM read and the main memory access each take a cycle, so each */
    /* beat is moved in a request/answer pair of states.                        */
    typedef enum logic [2:0] {
        S_IDLE,
        S_EVICT_READ,
        S_EVICT_WRITE,
        S_FILL_REQ,
        S_FILL_WRITE
    } state_t;

////////////////////////////////////////////////////////////////////////////////
//  Address decode
////////////////////////////////////////////////////////////////////////////////

    entry_t entries [NB_BLOCKS];

    tag_t tag;
    assign tag = address_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];

    line_idx_t line_idx;
    assign line_idx = address_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];

    offset_t offset;
    assign offset = address_i[OFFSET_WIDTH-1:0];

    logic hit;
    assign hit = entries[line_idx].valid && (entries[line_idx].tag == tag);

    logic miss;
    assign miss = !hit;

    /* The victim of a miss is the line currently held at the same index */
    logic victim_dirty;
    assign victim_dirty = entries[line_idx].valid && entries[line_idx].dirty;

////////////////////////////////////////////////////////////////////////////////
//  Miss handling registers
////////////////////////////////////////////////////////////////////////////////

    state_t state, next_state;

    /* The requested address is captured so the burst is immune to input changes */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [ADDR_WIDTH-1:0] address_r;
    /* verilator lint_on UNUSEDSIGNAL */

    /* Tag of the victim line, needed to address its write-back in main memory */
    tag_t evict_tag_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            address_r   <= '0;
            evict_tag_r <= '0;
        end
        else if (state == S_IDLE && ce_i && miss) begin
            address_r   <= address_i;
            evict_tag_r <= entries[line_idx].tag;
        end
    end

    line_idx_t line_idx_r;
    assign line_idx_r = address_r[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];

    tag_t tag_r;
    assign tag_r = address_r[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];

////////////////////////////////////////////////////////////////////////////////
//  Burst geometry for the current beat
////////////////////////////////////////////////////////////////////////////////

    /* Beat being moved within the current evict/fill burst */
    beat_idx_t beat_idx;

    logic last_beat;
    assign last_beat = (beat_idx == beat_idx_t'(BEATS_PER_LINE-1));

    /* Byte offset of the current beat within the line */
    offset_t beat_byte;
    assign beat_byte = offset_t'(beat_idx) << MEM_BYTE_W;

    /* Byte offset (within the line) of the cache word that holds this beat */
    offset_t word_byte;
    assign word_byte = beat_byte & ~offset_t'(DATA_BYTES-1);

    /* Position of the beat inside its cache word (DATA_BYTES is a power of two) */
    logic [LANE_BYTE_W-1:0] lane_byte;
    assign lane_byte = beat_byte[LANE_BYTE_W-1:0];

    logic [LANE_BIT_W-1:0] lane_bit;
    assign lane_bit = LANE_BIT_W'(lane_byte) << 3;

////////////////////////////////////////////////////////////////////////////////
//  Burst control
////////////////////////////////////////////////////////////////////////////////

    always_comb begin
        next_state = state;
        unique case (state)
            S_IDLE:
                if (ce_i && miss)
                    next_state = victim_dirty ? S_EVICT_READ : S_FILL_REQ;

            /* Read one beat-aligned word from the cache SRAM (answer next cycle) */
            S_EVICT_READ:
                next_state = S_EVICT_WRITE;

            /* Write that beat back to main memory */
            S_EVICT_WRITE:
                if (!mem_busy_i)
                    next_state = last_beat ? S_FILL_REQ : S_EVICT_READ;

            /* Request one beat from main memory (answer arrives next cycle) */
            S_FILL_REQ:
                if (!mem_busy_i)
                    next_state = S_FILL_WRITE;

            /* Store that beat into the cache SRAM */
            S_FILL_WRITE:
                next_state = last_beat ? S_IDLE : S_FILL_REQ;

            default:
                next_state = S_IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_idx <= '0;
        end
        else begin
            unique case (state)
                S_IDLE:
                    beat_idx <= '0;
                S_EVICT_WRITE:
                    if (!mem_busy_i)
                        beat_idx <= last_beat ? '0 : beat_idx + 1'b1;
                S_FILL_WRITE:
                    beat_idx <= last_beat ? '0 : beat_idx + 1'b1;
                default: ;
            endcase
        end
    end

////////////////////////////////////////////////////////////////////////////////
//  Cache line metadata
////////////////////////////////////////////////////////////////////////////////

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NB_BLOCKS; i++) begin
                entries[i].valid <= 1'b0;
                entries[i].dirty <= 1'b0;
                entries[i].tag   <= '0;
            end
        end
        else begin
            /* A write hit dirties the resident line */
            if (state == S_IDLE && ce_i && hit && (we_i != '0))
                entries[line_idx].dirty <= 1'b1;

            /* A completed fill validates the line and clears its dirty bit. */
            /* A pending write miss re-dirties it next cycle on the IDLE hit. */
            if (state == S_FILL_WRITE && last_beat) begin
                entries[line_idx_r].valid <= 1'b1;
                entries[line_idx_r].dirty <= 1'b0;
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
     * - During a miss being detected in IDLE
     * - During the whole evict/fill burst
     * Write-back hits never reach main memory, so they never stall.
    **/
    assign busy_o = (state != S_IDLE) || (ce_i && miss);

////////////////////////////////////////////////////////////////////////////////
//  Cache memory interface
////////////////////////////////////////////////////////////////////////////////

    always_comb begin
        /* Default: hold the SRAM idle, output stays stable */
        cache_ce_o   = 1'b0;
        cache_we_o   = '0;
        cache_addr_o = {line_idx_r, word_byte};
        cache_data_o = data_i;

        unique case (state)
            /* Serve a hit: read or write the addressed word */
            S_IDLE: begin
                cache_ce_o   = ce_i && hit;
                cache_we_o   = (ce_i && hit) ? we_i : '0;
                cache_addr_o = {line_idx, offset};
            end
            /* Read the victim word that holds the beat to be written back */
            S_EVICT_READ: begin
                cache_ce_o   = 1'b1;
            end
            /* Store the fetched beat into its lane of the cache word */
            S_FILL_WRITE: begin
                cache_ce_o                       = 1'b1;
                cache_we_o[lane_byte +: MEM_BYTES] = '1;
                cache_data_o[lane_bit +: MEM_WIDTH] = mem_data_i;
            end
            default: ;
        endcase
    end

////////////////////////////////////////////////////////////////////////////////
//  Main memory interface
////////////////////////////////////////////////////////////////////////////////

    always_comb begin
        mem_ce_o   = 1'b0;
        mem_we_o   = '0;
        mem_addr_o = {tag_r, line_idx_r, {OFFSET_WIDTH{1'b0}}} | ADDR_WIDTH'(beat_byte);

        unique case (state)
            /* Write the victim beat back to its old address */
            S_EVICT_WRITE: begin
                mem_ce_o   = 1'b1;
                mem_we_o   = '1;
                mem_addr_o = {evict_tag_r, line_idx_r, {OFFSET_WIDTH{1'b0}}} | ADDR_WIDTH'(beat_byte);
            end
            /* Fetch one beat of the new line */
            S_FILL_REQ: begin
                mem_ce_o   = 1'b1;
            end
            default: ;
        endcase
    end

    /* Data written back during an eviction is the beat's lane of the cache word */
    assign mem_data_o = cache_data_i[lane_bit +: MEM_WIDTH];

endmodule
