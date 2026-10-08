//------------------------------------------------------------------------------
// IVAN PALADIN        - 07/OUT/2026
// ANGELO DAL ZOTTO    - 08/OUT/2026
//------------------------------------------------------------------------------
// Simulation-only monitor for DMCtrl. Reads DMCtrl internals through upward
// references (DMCtrl.<signal>), so it only works bound into DMCtrl, with the
// same parameters as the bound instance:
//     bind DMCtrl DMDebug #(.ADDR_WIDTH(ADDR_WIDTH), .CACHE_WIDTH(CACHE_WIDTH),
//                           .OFFSET_WIDTH(OFFSET_WIDTH), .WMODE(WMODE)) u_debug ();
// Logs every FILL, EVICT and DIRTY event as CSV to ./debug/<instance path>.csv:
//     time,event,line,tag
// * GAPH - Hardware Design Support Group
// * PUCRS - Pontifical Catholic University of Rio Grande do Sul <https://pucrs.br/>
//------------------------------------------------------------------------------

module DMDebug
    import DMPkg::*;
#(
    /* Must be the bound DMCtrl's parameters (passed by the bind). They can't be read */
    /* as DMCtrl.<param>: constants can't be hierarchical (IEEE 1800 6.20.2), and     */
    /* each cache configuration needs its own DMDebug copy, or the widths read        */
    /* through DMCtrl.<signal> are fixed once for all caches. The widths are unused   */
    /* here, but they set the widths of DMCtrl.tag_r/line_idx_r, so they must still   */
    /* split the copies.                                                              */
    /* verilator lint_off UNUSEDPARAM */
    parameter int unsigned ADDR_WIDTH   = 20,
    parameter int unsigned CACHE_WIDTH  = 12,
    parameter int unsigned OFFSET_WIDTH = 6,
    /* verilator lint_on UNUSEDPARAM */
    parameter write_mode_t WMODE        = WRITE_BACK
);

    logic fill_done, evict_done, dirty_set;

    assign fill_done  = (DMCtrl.current_state == FILL ) && DMCtrl.end_fill;
    assign evict_done = (DMCtrl.current_state == EVICT) && DMCtrl.end_evict;
    assign dirty_set  = (DMCtrl.current_state == IDLE ) && (WMODE == WRITE_BACK)
                     &&  DMCtrl.is_write && !DMCtrl.miss && !DMCtrl.entries[DMCtrl.line_idx].dirty;

    string file_name;
    int    fd;

    /* One file per instance: icache and dcache must not share (and clobber) the same file */
    initial begin
        file_name = $sformatf("./debug/%m.csv");
        fd        = $fopen(file_name, "w");
        if (fd == 0)
            $display("[DMDebug] Could not open %s", file_name);
        else
            $fdisplay(fd, "time,event,line,tag");
    end

    always @(posedge DMCtrl.clk) begin
        if (fd != 0) begin
            if (fill_done)
                $fdisplay(fd, "%0d,FILL,%0d,%0d",  $time, DMCtrl.line_idx_r, DMCtrl.tag_r);
            if (evict_done)
                $fdisplay(fd, "%0d,EVICT,%0d,%0d", $time, DMCtrl.line_idx_r, DMCtrl.evict_tag_r);
            if (dirty_set)
                $fdisplay(fd, "%0d,DIRTY,%0d,%0d", $time, DMCtrl.line_idx, DMCtrl.tag);
        end
    end

    final begin
        if (fd != 0)
            $fclose(fd);
    end

endmodule
