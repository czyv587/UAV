################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
CMD/28335_RAM_lnk.exe: ../CMD/28335_RAM_lnk.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/bin/cl2000" -v28 -ml --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 -z -m"../CMD/Debug/LED.map" --heap_size=1000 --stack_size=1000 --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/lib" -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/include" -i"F:/workspace/LED" --reread_libs --xml_link_info="LED_linkInfo.xml" --rom_model -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMD/DSP2833x_Headers_nonBIOS.exe: ../CMD/DSP2833x_Headers_nonBIOS.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/bin/cl2000" -v28 -ml --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 -z -m"../CMD/Debug/LED.map" --heap_size=1000 --stack_size=1000 --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/lib" -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/include" -i"F:/workspace/LED" --reread_libs --xml_link_info="LED_linkInfo.xml" --rom_model -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


