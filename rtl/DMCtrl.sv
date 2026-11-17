//------------------------------------------------------------------------------
// FERNANDO MORAES     - 05/OUT/2025
// IVAN PALADIN JUNIOR - 16/MAR/2026
// ANGELO DAL ZOTTO    - 30/ABR/2026
//------------------------------------------------------------------------------
// Direct-mapped coherent cache controller with external memories.
// - Connects two memory buses:
//   (a) "cache_*" connects to a small SRAM used as the cache data store
//   (b) "a_*" to "e_*" connect to the main memory
// - Core side is an OBI subordinate: a hit is passed to the cache SRAM
// - Cache SRAM side is an OBI manager, any latency
// - Memory side is a TileLink 1.9.3 TL-C master, one Acquire, Release or Put
//   at a time, one 32-bit word per beat. Each line holds a permission:
//   N (invalid), B (read only, clean) or T (read and write, maybe dirty)
//     read miss:     A AcquireBlock NtoB  -> D GrantData beats -> E GrantAck
//     write miss:    A AcquireBlock NtoT  (a B line is dropped first, it is clean)
//     dirty victim:  C ReleaseData TtoN   -> D ReleaseAck (clean victims are dropped)
//     probe:         B ProbeBlock/Perm    -> C ProbeAck, or ProbeAckData if a
//                                            dirty line loses T on a ProbeBlock
//     write-through: A PutPartialData     -> D AccessAck (lines never get dirty)
//   The Grant may give more than asked (T for NtoB), the line keeps what it gives.
// - Probes are served in IDLE, in PUT, and in FILL until GrantData starts, as
//   the Grant may wait for them (spec 9.2). They wait during EVICT, a GrantData
//   burst and a pending GrantAck, which are higher priority messages in flight
//   (spec 5). GrantAck goes out on the last GrantData beat, before the SRAM
//   writes end, so it never waits behind a ProbeAck.
// - SRAM responses can't be stalled: line words read for channel C wait in
//   c_buf, which a read is only issued into when it has room. GrantData beats
//   wait for the SRAM in fill_buf, as a TileLink beat may be retracted and an
//   OBI request not. owners routes each SRAM response, in order, to the core,
//   to c_buf, or nowhere (fill writes).
// - Miss: fetch from RAM, fill cache, then grant (the request holds until gnt)
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

    /* Channel B: only ProbeBlock and ProbePerm, which carry no data */
    input  logic                        b_valid_i,
    output logic                        b_ready_o,
    input  logic [2:0]                  b_opcode_i,
    input  logic [2:0]                  b_param_i,
    input  logic [3:0]                  b_size_i,
    input  logic [SOURCE_WIDTH-1:0]     b_source_i,
    input  logic [ADDR_WIDTH-1:0]       b_address_i,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [3:0]                  b_mask_i,
    input  logic [31:0]                 b_data_i,
    input  logic                        b_corrupt_i,
    /* verilator lint_on UNUSEDSIGNAL */

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

    /* Channel D: one operation at a time, so the opcode is known */
    input  logic                        d_valid_i,
    output logic                        d_ready_o,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [2:0]                  d_opcode_i,
    input  logic [3:0]                  d_size_i,
    input  logic [SOURCE_WIDTH-1:0]     d_source_i,
    input  logic                        d_denied_i,
    input  logic                        d_corrupt_i,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [1:0]                  d_param_i,
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
        perm_t perm;
        logic  dirty;
        tag_t  tag;
    } entry_t;

    /* Owner of an SRAM response */
    typedef enum logic [1:0] {
        TO_NONE = 2'd0,
        TO_CORE = 2'd1,
        TO_C    = 2'd2
    } owner_t;

    localparam word_idx_t LAST_WORD = word_idx_t'(FILL_WORDS-1);

////////////////////////////////////////////////////////////////////////////////
//  Signals
////////////////////////////////////////////////////////////////////////////////

    /* Core requests */
    logic       read_ok;
    logic       write_ok;
    logic       need_fill;
    logic       dirty_victim;
    logic       leave_idle;
    logic       put_hit;
    logic       hit_access;
    logic       put_access;
    logic       core_access;
    logic       core_waiting;
    logic       put_written;
    logic       dirty_set;
    tag_t       tag;
    tag_t       evict_tag;
    offset_t    offset;
    line_idx_t  line_idx;
    entry_t     line;
    fsm_t       next_state;
    fsm_t       current_state;
    entry_t     entries [NB_BLOCKS];

    /* Fill */
    logic       filling;
    logic       acquire_sent;
    logic       grant_beat;
    logic       granting;
    logic       fill_push;
    logic       fill_room;
    logic       fill_valid;
    logic       fill_write;
    logic       end_fill;
    logic       e_pending;
    logic [31:0] fill_data;
    perm_t      grant_perm;
    word_idx_t  grant_count;
    word_idx_t  fill_count;

    /* Probes */
    logic       probe_busy;
    logic       probe_accept;
    logic       probe_hit;
    logic       probe_data;
    tag_t       b_tag;
    line_idx_t  b_index;
    entry_t     b_line;
    perm_t      probe_cap;
    perm_t      probe_old;
    perm_t      probe_new;
    logic [2:0] probe_report;
    probe_fsm_t probe_state;
    logic [2:0] probe_param;
    logic [3:0] probe_size;
    logic [SOURCE_WIDTH-1:0] probe_source;
    logic [ADDR_WIDTH-1:0]   probe_address;

    /* Line words to channel C */
    logic       evicting;
    logic       streaming;
    logic       stream_read;
    logic       stream_beat;
    logic       end_stream;
    logic       end_evict;
    logic       reads_done;
    logic       release_sent;
    logic       acked;
    logic       c_push;
    logic       c_room;
    logic       c_almost_full;
    logic       c_credit;
    logic       c_buf_valid;
    logic [1:0] c_inflight;
    line_idx_t  stream_index;
    word_idx_t  read_count;
    word_idx_t  send_count;

    /* Cache SRAM */
    logic       sram_issue;
    logic       stream_pick;
    logic       stream_held;
    logic       stream_sent;
    logic       fill_sent;
    logic       owner_room;
    logic       owner_valid;
    owner_t     owner_in;
    owner_t     owner_out;

////////////////////////////////////////////////////////////////////////////////
//  Core requests
////////////////////////////////////////////////////////////////////////////////

    /* OBI holds addr_i until gnt, which only comes after the fill: */
    /* fill and evict can use addr_i without registering it         */
    assign tag       = addr_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign offset    = addr_i[OFFSET_WIDTH-1:0];
    assign line_idx  = addr_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign line      = entries[line_idx];
    assign evict_tag = line.tag;

    /* Write-through writes only need to read the line: the write goes to memory */
    assign read_ok      = (line.perm != PERM_N) && (line.tag == tag);
    assign write_ok     = (WMODE == WRITE_THROUGH) ? read_ok : (line.perm == PERM_T) && (line.tag == tag);
    assign need_fill    = req_i && !(we_i ? write_ok : read_ok);
    assign dirty_victim = line.dirty;

    /* Line transfers start with no probe and no GrantAck in flight */
    assign leave_idle   = (current_state == IDLE) && need_fill && !probe_busy && !e_pending;

    /* The SRAM serves the core in IDLE and PUT, while no probe is in progress.  */
    /* A presented probe goes first, unless a core request is already waiting.   */
    /* Write-through writes reach the SRAM in PUT, after their PutPartialData.   */
    assign put_hit      = (WMODE == WRITE_THROUGH) && req_i && we_i && write_ok
                       && (current_state == IDLE) && !probe_busy;
    assign hit_access   = req_i && !need_fill && !(we_i && WMODE == WRITE_THROUGH)
                       && (current_state == IDLE) && !probe_busy && (core_waiting || !b_valid_i);
    assign put_access   = req_i && (current_state == PUT) && !put_written;
    assign core_access  = hit_access || put_access;
    assign dirty_set    = (WMODE == WRITE_BACK) && hit_access && we_i && sram_issue && !line.dirty;

    always_comb begin
        unique case (current_state)
            IDLE:    begin
                if (leave_idle)
                    next_state = dirty_victim ? EVICT : FILL;
                else if (put_hit && a_ready_i)
                    next_state = PUT;
                else
                    next_state = IDLE;
            end
            FILL:    next_state = end_fill ? IDLE : FILL;
            EVICT:   next_state = ((release_sent || end_evict) && (acked || d_valid_i)) ? IDLE : EVICT;
            PUT:     next_state = ((put_written || (put_access && sram_issue)) && (acked || d_valid_i)) ? IDLE : PUT;
            default: next_state = IDLE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    /* A core SRAM request not yet granted must hold */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            core_waiting <= 1'b0;
        else
            core_waiting <= core_access && !sram_issue;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            put_written <= 1'b0;
        else
            put_written <= (current_state == PUT) && (put_written || (put_access && sram_issue));
    end

////////////////////////////////////////////////////////////////////////////////
//  Fill: AcquireBlock, GrantData beats through fill_buf, GrantAck
////////////////////////////////////////////////////////////////////////////////

    assign filling    = (current_state == FILL);
    assign fill_push  = filling && d_valid_i;
    assign grant_beat = fill_push && fill_room;
    assign end_fill   = fill_sent && (fill_count == LAST_WORD);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            acquire_sent <= 1'b0;
        else
            acquire_sent <= filling && (acquire_sent || a_ready_i);
    end

    /* A GrantData burst started: probes wait until the fill ends */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            granting <= 1'b0;
        else
            granting <= filling && !end_fill && (granting || grant_beat);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            grant_count <= '0;
        else if (!filling)
            grant_count <= '0;
        else
            grant_count <= grant_count + word_idx_t'(grant_beat);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fill_count <= '0;
        else if (!filling)
            fill_count <= '0;
        else
            fill_count <= fill_count + word_idx_t'(fill_sent);
    end

    /* GrantData caps the permission: toT gives T, anything else B */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            grant_perm <= PERM_N;
        else if (grant_beat)
            grant_perm <= (d_param_i == 2'(TL_CAP_TOT)) ? PERM_T : PERM_B;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            e_sink_o <= '0;
        else if (grant_beat)
            e_sink_o <= d_sink_i;
    end

    /* GrantAck goes out on the last GrantData beat, even while the SRAM writes go on */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            e_pending <= 1'b0;
        else
            e_pending <= (e_pending && !e_ready_i) || (grant_beat && grant_count == LAST_WORD);
    end

    assign e_valid_o = e_pending;

    /* No fall through: the SRAM request must hold even if d_valid is retracted */
    /* verilator lint_off PINCONNECTEMPTY */
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
        .tx_ack_i      (fill_sent   ),
        .data_o        (fill_data   ),
        .count_o       (            ),
        .almost_full_o (            ),
        .almost_empty_o(            )
    );
    /* verilator lint_on PINCONNECTEMPTY */

////////////////////////////////////////////////////////////////////////////////
//  Probes
////////////////////////////////////////////////////////////////////////////////

    assign probe_busy = (probe_state != P_IDLE);

    always_comb begin
        if (probe_busy)
            b_ready_o = 1'b0;
        else begin
            unique case (current_state)
                IDLE:    b_ready_o = !core_waiting && !leave_idle && !e_pending;
                FILL:    b_ready_o = !granting && !e_pending;
                PUT:     b_ready_o = 1'b1;
                default: b_ready_o = 1'b0;
            endcase
        end
    end

    assign probe_accept = b_valid_i && b_ready_o;

    assign b_tag     = b_address_i[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    assign b_index   = b_address_i[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];
    assign b_line    = entries[b_index];
    assign probe_hit = (b_line.perm != PERM_N) && (b_line.tag == b_tag);
    assign probe_old = probe_hit ? b_line.perm : PERM_N;

    always_comb begin
        unique case (b_param_i)
            TL_CAP_TOT: probe_cap = PERM_T;
            TL_CAP_TOB: probe_cap = PERM_B;
            default:    probe_cap = PERM_N;
        endcase
    end

    assign probe_new  = (probe_old < probe_cap) ? probe_old : probe_cap;

    /* A dirty line losing T is written back, unless ProbePerm says the data is not needed */
    assign probe_data = probe_hit && b_line.dirty && (probe_new != PERM_T)
                     && (b_opcode_i == TL_B_PROBE_BLOCK);

    /* Shrink or Report from the old to the new permission */
    always_comb begin
        unique case ({probe_old, probe_new})
            {PERM_T, PERM_T}: probe_report = TL_REPORT_TTOT;
            {PERM_T, PERM_B}: probe_report = TL_PRUNE_TTOB;
            {PERM_T, PERM_N}: probe_report = TL_PRUNE_TTON;
            {PERM_B, PERM_B}: probe_report = TL_REPORT_BTOB;
            {PERM_B, PERM_N}: probe_report = TL_PRUNE_BTON;
            default:          probe_report = TL_REPORT_NTON;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            probe_state <= P_IDLE;
        else begin
            unique case (probe_state)
                P_IDLE:  if (probe_accept) probe_state <= probe_data ? P_DATA : P_ACK;
                P_ACK:   if (c_ready_i)    probe_state <= P_IDLE;
                P_DATA:  if (end_stream)   probe_state <= P_IDLE;
                default:                   probe_state <= P_IDLE;
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            probe_param <= '0;
        else if (probe_accept)
            probe_param <= probe_report;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            probe_size <= '0;
        else if (probe_accept)
            probe_size <= b_size_i;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            probe_source <= '0;
        else if (probe_accept)
            probe_source <= b_source_i;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            probe_address <= '0;
        else if (probe_accept)
            probe_address <= b_address_i;
    end

////////////////////////////////////////////////////////////////////////////////
//  Line words to channel C: ReleaseData (evict) or ProbeAckData (probe)
////////////////////////////////////////////////////////////////////////////////

    /* Evictions and probe write backs never overlap: probes wait during EVICT, */
    /* and EVICT starts with no probe in progress. EVICT may outlast its last   */
    /* beat, waiting for ReleaseAck: the stream ends with the beat.             */
    assign evicting     = (current_state == EVICT);
    assign streaming    = (evicting && !release_sent) || (probe_state == P_DATA);
    assign stream_index = evicting ? line_idx : probe_address[ADDR_WIDTH-TAG_WIDTH-1:ADDR_WIDTH-TAG_WIDTH-INDEX_WIDTH];

    /* A read is issued only if c_buf has room for it and for the one in flight */
    assign c_credit     = (c_inflight == 2'd0) ? c_room
                        : (c_inflight == 2'd1) && !c_almost_full;
    assign stream_read  = streaming && !reads_done && c_credit;
    assign stream_beat  = streaming && c_valid_o && c_ready_i;
    assign end_stream   = stream_beat && (send_count == LAST_WORD);
    assign end_evict    = evicting && end_stream;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            read_count <= '0;
        else if (!streaming)
            read_count <= '0;
        else
            read_count <= read_count + word_idx_t'(stream_sent);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            send_count <= '0;
        else if (!streaming)
            send_count <= '0;
        else
            send_count <= send_count + word_idx_t'(stream_beat);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reads_done <= 1'b0;
        else
            reads_done <= streaming && !end_stream && (reads_done || (stream_sent && read_count == LAST_WORD));
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            c_inflight <= '0;
        else
            c_inflight <= c_inflight + 2'(stream_sent) - 2'(c_push);
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

    /* verilator lint_off PINCONNECTEMPTY */
    RingBuffer #(
        .DATA_SIZE              (32),
        .BUFFER_SIZE            (2 ),
        .FIRST_WORD_FALL_THROUGH(1 ),
        .ALMOST_FULL_THRESHOLD  (1 ),
        .ALMOST_EMPTY_THRESHOLD (0 )
    ) c_buf (
        .clk_i         (clk          ),
        .rst_ni        (rst_n        ),
        .buf_rst_i     (1'b0         ),
        .rx_i          (c_push       ),
        .rx_ack_o      (c_room       ),
        .data_i        (cache_rdata_i),
        .tx_o          (c_buf_valid  ),
        .tx_ack_i      (c_ready_i    ),
        .data_o        (c_data_o     ),
        .count_o       (             ),
        .almost_full_o (c_almost_full),
        .almost_empty_o(             )
    );
    /* verilator lint_on PINCONNECTEMPTY */

////////////////////////////////////////////////////////////////////////////////
//  Line permissions
////////////////////////////////////////////////////////////////////////////////

    /* Same-cycle updates never hit the same line: a probe is not accepted when */
    /* IDLE is left, during EVICT or a GrantData burst, or with a core write    */
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NB_BLOCKS; i++) begin
                entries[i].perm  <= PERM_N;
                entries[i].dirty <= 1'b0;
                entries[i].tag   <= '0;
            end
        end
        else begin
            if (probe_accept && probe_hit) begin
                entries[b_index].perm  <= probe_new;
                entries[b_index].dirty <= b_line.dirty && (probe_new == PERM_T);
            end

            if (dirty_set)
                entries[line_idx].dirty <= 1'b1;

            /* A clean line is dropped when its slot is filled */
            if (leave_idle && !dirty_victim) begin
                entries[line_idx].perm  <= PERM_N;
                entries[line_idx].dirty <= 1'b0;
            end

            if (end_fill) begin
                entries[line_idx].perm  <= grant_perm;
                entries[line_idx].dirty <= 1'b0;
                entries[line_idx].tag   <= tag;
            end

            if (end_evict) begin
                entries[line_idx].perm  <= PERM_N;
                entries[line_idx].dirty <= 1'b0;
            end
        end
    end

////////////////////////////////////////////////////////////////////////////////
//  Cache SRAM
////////////////////////////////////////////////////////////////////////////////

    /* Only fill writes and probe reads share the SRAM (FILL before GrantData).  */
    /* Fill writes go first, but a read request waiting for gnt must hold.       */
    /* No request starts while owners is full; none can be waiting then.         */
    assign fill_write  = filling && fill_valid;
    assign stream_pick = stream_read && (stream_held || !fill_write);

    always_comb begin
        if (core_access) begin
            cache_req_o   = owner_room;
            cache_addr_o  = {line_idx, offset};
            cache_we_o    = we_i;
            cache_be_o    = be_i;
            cache_wdata_o = wdata_i;
            owner_in      = TO_CORE;
        end
        else if (stream_pick) begin
            cache_req_o   = owner_room;
            cache_addr_o  = {stream_index, read_count, 2'b00};
            cache_we_o    = 1'b0;
            cache_be_o    = 4'hF;
            cache_wdata_o = fill_data;
            owner_in      = TO_C;
        end
        else begin
            cache_req_o   = fill_write && owner_room;
            cache_addr_o  = {line_idx, fill_count, 2'b00};
            cache_we_o    = 1'b1;
            cache_be_o    = 4'hF;
            cache_wdata_o = fill_data;
            owner_in      = TO_NONE;
        end
    end

    assign sram_issue  = cache_req_o && cache_gnt_i;
    assign stream_sent = sram_issue && (owner_in == TO_C);
    assign fill_sent   = sram_issue && (owner_in == TO_NONE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stream_held <= 1'b0;
        else
            stream_held <= stream_pick && !sram_issue;
    end

    /* One owner per SRAM access in flight: at most 4 */
    /* verilator lint_off PINCONNECTEMPTY */
    RingBuffer #(
        .DATA_SIZE              (2),
        .BUFFER_SIZE            (4),
        .FIRST_WORD_FALL_THROUGH(0),
        .ALMOST_FULL_THRESHOLD  (0),
        .ALMOST_EMPTY_THRESHOLD (0)
    ) owners (
        .clk_i         (clk           ),
        .rst_ni        (rst_n         ),
        .buf_rst_i     (1'b0          ),
        .rx_i          (sram_issue    ),
        .rx_ack_o      (owner_room    ),
        .data_i        (owner_in      ),
        .tx_o          (owner_valid   ),
        .tx_ack_i      (cache_rvalid_i),
        .data_o        (owner_out     ),
        .count_o       (              ),
        .almost_full_o (              ),
        .almost_empty_o(              )
    );
    /* verilator lint_on PINCONNECTEMPTY */

    assign c_push = cache_rvalid_i && owner_valid && (owner_out == TO_C);

////////////////////////////////////////////////////////////////////////////////
//  Memory interface
////////////////////////////////////////////////////////////////////////////////

    /* Channel A: AcquireBlock of the missing line, or PutPartialData of a write-through hit */
    assign a_valid_o   = put_hit || (filling && !acquire_sent);
    assign a_opcode_o  = filling ? TL_A_ACQUIRE_BLOCK : TL_A_PUT_PARTIAL_DATA;
    assign a_size_o    = filling ? 4'(OFFSET_WIDTH)   : 4'd2;
    assign a_source_o  = '0;
    assign a_address_o = filling ? {tag, line_idx, offset_t'(0)} : {addr_i[ADDR_WIDTH-1:2], 2'b00};
    assign a_mask_o    = filling ? 4'hF : be_i;
    assign a_data_o    = wdata_i;
    assign a_corrupt_o = 1'b0;

    always_comb begin
        if (!filling)
            a_param_o = 3'd0;
        else if ((WMODE == WRITE_BACK) && we_i)
            a_param_o = TL_GROW_NTOT;
        else
            a_param_o = TL_GROW_NTOB;
    end

    /* Channel C: ReleaseData or ProbeAckData beats from c_buf, or a ProbeAck */
    assign c_valid_o   = streaming ? c_buf_valid : (probe_state == P_ACK);
    assign c_corrupt_o = 1'b0;

    always_comb begin
        if (evicting) begin
            c_opcode_o  = TL_C_RELEASE_DATA;
            c_param_o   = TL_PRUNE_TTON;
            c_size_o    = 4'(OFFSET_WIDTH);
            c_source_o  = '0;
            c_address_o = {evict_tag, line_idx, offset_t'(0)};
        end
        else begin
            c_opcode_o  = (probe_state == P_DATA) ? TL_C_PROBE_ACK_DATA : TL_C_PROBE_ACK;
            c_param_o   = probe_param;
            c_size_o    = probe_size;
            c_source_o  = probe_source;
            c_address_o = probe_address;
        end
    end

    /* Channel D: GrantData beats while fill_buf has room; acks always */
    /* ponytail: d_denied and d_corrupt are ignored, add error reporting when a slave can deny */
    assign d_ready_o = filling ? fill_room : 1'b1;

////////////////////////////////////////////////////////////////////////////////
//  Core interface
////////////////////////////////////////////////////////////////////////////////

    /**
     * The SRAM serves core accesses directly. The request waits:
     * - During a miss, until the line is filled (and evicted first if dirty)
     * - During a probe
     * - Write-through: on write hits, until the memory accepts the PutPartialData
    **/
    assign gnt_o    = core_access && sram_issue;
    assign rvalid_o = cache_rvalid_i && owner_valid && (owner_out == TO_CORE);
    assign rdata_o  = cache_rdata_i;

endmodule
