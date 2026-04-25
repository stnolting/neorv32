# ***********************************************
# read out NEORV32 trace buffer and show the according code points
# ***********************************************
define tracer_get

  set $tracer_base = 0xFFF30000
  set $tracer_ctrl = $tracer_base + 0x0
  set $tracer_delta_src = $tracer_base + 0x8
  set $tracer_delta_dst = $tracer_base + 0xc
  set $i = 0

  # trace data available?
  set $ctrl = *((unsigned int*)($tracer_ctrl))
  set $ctrl = $ctrl & (1 << 5)

  if $ctrl == 0
    printf "No trace data available.\n"
  else
    while $ctrl != 0
      printf "---------------------------\n"

      # get trace data
      set $delta_src = *((unsigned int*)($tracer_delta_src))
      set $delta_dst = *((unsigned int*)($tracer_delta_dst))
      set $src = $delta_src & 0xfffffffe
      set $dst = $delta_dst & 0xfffffffe

      # print branch source and destination addresses
      printf "[%d] SRC: 0x%x -> DST: 0x%x", $i, $src , $dst
      set $i = $i + 1

      # check if this is the very first trace packet
      if ($delta_src & 1)
        printf " <TRACE_START>"
      end
      # check if branch was caused by a trap
      if ($delta_dst & 1)
        printf " <TRAP_ENTRY>"
      end
      printf "\n"

      # print according source code lines
      # this requires that debug symbols are available in the ELF
      info line *$src
      info line *$dst

      # trace data available?
      set $ctrl = *((unsigned int*)($tracer_ctrl))
      set $ctrl = $ctrl & (1 << 5)
    end
  end
end


# ***********************************************
# (re-)start the NEORV32 tracer in free-running
# argument: CPU select (0 or 1)
# ***********************************************
define tracer_start

  set $hart = $arg0 & 1

  printf "Tracer started for CPU %d\n", $hart

  set $tracer_base = 0xFFF30000
  set $tracer_ctrl = $tracer_base + 0x0
  set $tracer_stop = $tracer_base + 0x4

  # reset tracer
  set *((unsigned int*)($tracer_ctrl)) = 0

  # not stop address
  set *((unsigned int*)($tracer_stop)) = -1

  # enable tracer and start
  set *((unsigned int*)($tracer_ctrl)) = 0b101 + ($hart << 1)

end
