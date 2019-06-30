#!/bin/bash

echo "Incorporating HLS generated HDL sources into project..";

for prj_template in prj/*.template; do

#Extract module number from filename
module_number="$(cut -d'_' -f2 <<<"$prj_template")";
#Remove trailing ".template" to get target file name
prj_file=${prj_template/.template/}

#copy template to target file
cp $prj_template $prj_file

#delete all existing (old) symbolic links in ReconOS source dir
find src/rt_module_$module_number/vhdl -type l -delete

#check if directory with HLS generated HDL code exists
if [ -d "src/rt_module_"$module_number"/hls_module/solution1/impl/ip/hdl/vhdl" ]; then

 #loop trough HLS generated HDL sources
 for hdl_source in src/rt_module_$module_number/hls_module/solution1/impl/ip/hdl/vhdl/*.vhd; do
 
	#extract filename of sourcefile
	hdl_source_filename=$(basename $hdl_source);
	
	#add each sourcefile to project file (referenced from usual ReconOS source dir via link)
	echo "vhdl rt_reconf_v1_00_a ./src/rt_module_"$module_number"/vhdl/"$hdl_source_filename"" >> $prj_file;
	
	#create symbolic link for each sourcefile
	ln -s ../hls_module/solution1/impl/ip/hdl/vhdl/$hdl_source_filename src/rt_module_$module_number/vhdl/$hdl_source_filename
	
 done
fi
done

#Run main PR flow script
source ../../scripts/setup_env.sh
vivado -mode batch -source ./scripts/design.tcl -notrace