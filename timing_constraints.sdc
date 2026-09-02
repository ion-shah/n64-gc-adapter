# ==============================================================================
# Clock Constraints
# timing_constraints.sdc
# ==============================================================================
# Define the 66MHz clock. 
# -name assigns an internal name to the clock.
# -period is in nanoseconds.
# [get_ports clk] targets the physical top-level input pin named 'clk'.

set clk_freq_mhz 66.0
create_clock -name clk_66mhz -period [expr {1000.0 / $clk_freq_mhz}] [get_ports clk]

create_generated_clock -name clk_core -source [get_ports clk] -divide_by 3 [get_registers {*|clk_core}]


# (Optional but recommended for Intel/Altera Quartus) 
# This command automatically calculates and applies internal clock jitter and margin.
derive_clock_uncertainty

# ==============================================================================
# I/O Constraints
# ==============================================================================
# The N64 JoyBus line is asynchronous. It feeds directly into a synchronizer.
# We tell the timing analyzer to completely ignore the timing path from this pin.
set_false_path -from [get_ports data_in] -to [all_registers]
