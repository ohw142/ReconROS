###############################################################
###   Tcl Variables
###############################################################
####Define location for "Tcl" directory. Defaults to "./Tcl"
set tclHome "./scripts/tcl"
if {[file exists $tclHome]} {
   set tclDir $tclHome
} elseif {[file exists "./tcl"]} {
   set tclDir  "./tcl"
} else {
   error "ERROR: No valid location found for required Tcl scripts. Set \$tclDir in design.tcl to a valid location."
}
puts "Setting TCL dir to $tclDir"

####Source required Tcl Procs
source $tclDir/design_utils.tcl
source $tclDir/log_utils.tcl
source $tclDir/synth_utils.tcl
source $tclDir/impl_utils.tcl
source $tclDir/hd_floorplan_utils.tcl

###############################################################
### Define Part, Package, Speedgrade 
###############################################################
set device       "xc7z020"
set package      "clg400"
set speed        "-1"
set part         $device$package$speed
check_part $part

###############################################################
###  Setup Variables
###############################################################
#set tclParams [list <param1> <value> <param2> <value> ... <paramN> <value>]
set tclParams [list hd.visual 1 \
              ]

####flow control
set run.rmSynth        1
set run.prImpl         1
set run.prVerify       1
set run.writeBitstream 1
set run.flatImpl       0

####Report and DCP controls - values: 0-required min; 1-few extra; 2-all
set verbose      1
set dcpLevel     1

####Output Directories
set synthDir  "./Synth"
set implDir   "./Implement"
set dcpDir    "./Checkpoint"
set bitDir    "./Bitstreams"

####Input Directories, not used in this demo. Paths directly point to the files in the build.hw and src folders.
set srcDir     "./src"
set xdcDir     "./constr"
set rtlDir     "./src/application"
#set ipDir      ""
set prjDir     "./prj"

###############################################################
### Top Definition
###############################################################
set top "design_1_wrapper"
set static "static"
add_module $static
set_attribute module $static moduleName      $top
set_attribute module $static top_level       1
set_attribute module $static synthCheckpoint ./build.hw/myReconOS.runs/synth_1/design_1_wrapper.dcp

####################################################################
### RP Module Definitions
####################################################################
set module1 "design_1_slot_0_0"

set module1_variant1 "module_1_hwt0"
set variant $module1_variant1
add_module $variant
set_attribute module $variant moduleName   $module1
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module1_variant2 "module_2_hwt0"
set variant $module1_variant2
add_module $variant
set_attribute module $variant moduleName   $module1
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module1_variant3 "module_3_hwt0"
set variant $module1_variant3
add_module $variant
set_attribute module $variant moduleName   $module1
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module1_variant4 "module_4_hwt0"
set variant $module1_variant4
add_module $variant
set_attribute module $variant moduleName   $module1
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module1_inst1 "design_1_i/slot_0"

####################################################################
### RP Module Definitions
####################################################################
set module2 "design_1_slot_1_0"

set module2_variant1 "module_1_hwt1"
set variant $module2_variant1
add_module $variant
set_attribute module $variant moduleName   $module2
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module2_variant2 "module_2_hwt1"
set variant $module2_variant2
add_module $variant
set_attribute module $variant moduleName   $module2
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module2_variant3 "module_3_hwt1"
set variant $module2_variant3
add_module $variant
set_attribute module $variant moduleName   $module2
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module2_variant4 "module_4_hwt1"
set variant $module2_variant4
add_module $variant
set_attribute module $variant moduleName   $module2
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module2_inst1 "design_1_i/slot_1"

####################################################################
### RP Module Definitions
####################################################################
set module3 "design_1_slot_2_0"

set module3_variant1 "module_1_hwt2"
set variant $module3_variant1
add_module $variant
set_attribute module $variant moduleName   $module3
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module3_variant2 "module_2_hwt2"
set variant $module3_variant2
add_module $variant
set_attribute module $variant moduleName   $module3
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module3_variant3 "module_3_hwt2"
set variant $module3_variant3
add_module $variant
set_attribute module $variant moduleName   $module3
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module3_variant4 "module_4_hwt2"
set variant $module3_variant4
add_module $variant
set_attribute module $variant moduleName   $module3
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module3_inst1 "design_1_i/slot_2"

####################################################################
### RP Module Definitions
####################################################################
set module4 "design_1_slot_3_0"

set module4_variant1 "module_1_hwt3"
set variant $module4_variant1
add_module $variant
set_attribute module $variant moduleName   $module4
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module4_variant2 "module_2_hwt3"
set variant $module4_variant2
add_module $variant
set_attribute module $variant moduleName   $module4
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module4_variant3 "module_3_hwt3"
set variant $module4_variant3
add_module $variant
set_attribute module $variant moduleName   $module4
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module4_variant4 "module_4_hwt3"
set variant $module4_variant4
add_module $variant
set_attribute module $variant moduleName   $module4
set_attribute module $variant prj          $prjDir/$variant.prj
#set_attribute module $variant xdc          $ipDir/$variant/constraints/image_filter_ooc.xdc
set_attribute module $variant synth        ${run.rmSynth}

set module4_inst1 "design_1_i/slot_3"

########################################################################
### Configuration (Implementation) Definition - Replicate for each Config
########################################################################

set config "config_module_1" 

add_implementation $config
set_attribute impl $config top             $top
set_attribute impl $config pr.impl         1
set_attribute impl $config implXDC         [list ./build.hw/myReconOS.srcs/constrs_1/new/design_1_pblocks.xdc \
												 ./build.hw/constraints/board.xdc \
                                           ]
set_attribute impl $config impl            ${run.prImpl} 
set_attribute impl $config partitions      [list [list $static           $top           implement] \
                                                 [list $module1_variant1 $module1_inst1 implement] \
                                                 [list $module2_variant1 $module2_inst1 implement] \
												 [list $module3_variant1 $module3_inst1 implement] \
												 [list $module4_variant1 $module4_inst1 implement] \
                                           ]
set_attribute impl $config verify          ${run.prVerify} 
set_attribute impl $config bitstream       ${run.writeBitstream} 
set_attribute impl $config cfgmem.pcap     1

########################################################################
### Configuration (Implementation) Definition - Replicate for each Config
########################################################################

set config "config_module_2" 

add_implementation $config
set_attribute impl $config top             $top
set_attribute impl $config pr.impl         1
set_attribute impl $config implXDC         [list ./build.hw/myReconOS.srcs/constrs_1/new/design_1_pblocks.xdc
                                           ]
set_attribute impl $config impl            ${run.prImpl} 
set_attribute impl $config partitions      [list [list $static           $top           import]    \
                                                 [list $module1_variant2 $module1_inst1 implement] \
                                                 [list $module2_variant2 $module2_inst1 implement] \
												 [list $module3_variant2 $module3_inst1 implement] \
												 [list $module4_variant2 $module4_inst1 implement] \
                                           ]
set_attribute impl $config verify          ${run.prVerify} 
set_attribute impl $config bitstream       ${run.writeBitstream}
set_attribute impl $config cfgmem.pcap     1

########################################################################
### Configuration (Implementation) Definition - Replicate for each Config
########################################################################

set config "config_module_3" 

add_implementation $config
set_attribute impl $config top             $top
set_attribute impl $config pr.impl         1
set_attribute impl $config implXDC         [list ./build.hw/myReconOS.srcs/constrs_1/new/design_1_pblocks.xdc
                                           ]
set_attribute impl $config impl            ${run.prImpl} 
set_attribute impl $config partitions      [list [list $static           $top           import]    \
                                                 [list $module1_variant3 $module1_inst1 implement] \
                                                 [list $module2_variant3 $module2_inst1 implement] \
												 [list $module3_variant3 $module3_inst1 implement] \
												 [list $module4_variant3 $module4_inst1 implement] \
                                           ]
set_attribute impl $config verify          ${run.prVerify} 
set_attribute impl $config bitstream       ${run.writeBitstream}
set_attribute impl $config cfgmem.pcap     1

########################################################################
### Configuration (Implementation) Definition - Replicate for each Config
########################################################################

set config "config_module_4" 

add_implementation $config
set_attribute impl $config top             $top
set_attribute impl $config pr.impl         1
set_attribute impl $config implXDC         [list ./build.hw/myReconOS.srcs/constrs_1/new/design_1_pblocks.xdc
                                           ]
set_attribute impl $config impl            ${run.prImpl} 
set_attribute impl $config partitions      [list [list $static           $top           import]    \
                                                 [list $module1_variant4 $module1_inst1 implement] \
                                                 [list $module2_variant4 $module2_inst1 implement] \
												 [list $module3_variant4 $module3_inst1 implement] \
												 [list $module4_variant4 $module4_inst1 implement] \
                                           ]
set_attribute impl $config verify          ${run.prVerify} 
set_attribute impl $config bitstream       ${run.writeBitstream}
set_attribute impl $config cfgmem.pcap     1

####################################################################
### Create Flat implementation run 
####################################################################

add_implementation Flat
set_attribute impl Flat top          $top
set_attribute impl Flat implXDC      [list ./build.hw/myReconOS.srcs/constrs_1/new/design_1_pblocks.xdc \
										   ./build.hw/constraints/board.xdc \					
                                     ]
set_attribute impl Flat partitions   [list [list $static           $top           implement] \
                                           [list $module1_variant1 $module1_inst1 implement] \
                                           [list $module2_variant1 $module2_inst1 implement] \
										   [list $module3_variant1 $module3_inst1 implement] \
										   [list $module4_variant1 $module4_inst1 implement] \
                                     ]
set_attribute impl Flat impl         ${run.flatImpl}

########################################################################
### Task / flow portion
########################################################################
source $tclDir/run.tcl

exit
