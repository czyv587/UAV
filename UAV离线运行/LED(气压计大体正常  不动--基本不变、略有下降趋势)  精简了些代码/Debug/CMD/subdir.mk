################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../CMD/28335_RAM_lnk.cmd \
../CMD/DSP2833x_Headers_nonBIOS.cmd 


# Each subdirectory must supply rules for building sources it contributes
CMD/28335_RAM_lnk.out: ../CMD/28335_RAM_lnk.cmd $(OBJS)
	@echo 'Building file: $<'
	@echo 'Invoking: Linker'
	$(shell echo --silicon_version=28 > ccsLinker.opt)
	$(shell echo -g >> ccsLinker.opt)
	$(shell echo --define="_DEBUG" --define="LARGE_MODEL" >> ccsLinker.opt)
	$(shell echo --diag_warning=225 >> ccsLinker.opt)
	$(shell echo --large_memory_model >> ccsLinker.opt)
	$(shell echo --float_support=fpu32 >> ccsLinker.opt)
	$(shell echo -z >> ccsLinker.opt)
	$(shell echo -m"../CMD/Debug/LED.map" >> ccsLinker.opt)
	$(shell echo --stack_size=1000 >> ccsLinker.opt)
	$(shell echo --heap_size=1000 >> ccsLinker.opt)
	$(shell echo --warn_sections >> ccsLinker.opt)
	$(shell echo -i"C:/Program Files (x86)/Texas Instruments/ccsv4/tools/compiler/c2000/lib" -i"C:/Program Files (x86)/Texas Instruments/ccsv4/tools/compiler/c2000/include" -i"C:/Users/Administrator/Desktop/LED" >> ccsLinker.opt)
	$(shell echo --reread_libs >> ccsLinker.opt)
	$(shell echo --rom_model >> ccsLinker.opt)
	$(shell echo $< >> ccsLinker.opt)
	$(shell type ccsObjs.opt >> ccsLinker.opt)
	"C:/Program Files (x86)/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" -@ccsLinker.opt -o "$@"
	@echo 'Finished building: $<'
	@echo ' '

CMD/DSP2833x_Headers_nonBIOS.out: ../CMD/DSP2833x_Headers_nonBIOS.cmd $(OBJS)
	@echo 'Building file: $<'
	@echo 'Invoking: Linker'
	$(shell echo --silicon_version=28 > ccsLinker.opt)
	$(shell echo -g >> ccsLinker.opt)
	$(shell echo --define="_DEBUG" --define="LARGE_MODEL" >> ccsLinker.opt)
	$(shell echo --diag_warning=225 >> ccsLinker.opt)
	$(shell echo --large_memory_model >> ccsLinker.opt)
	$(shell echo --float_support=fpu32 >> ccsLinker.opt)
	$(shell echo -z >> ccsLinker.opt)
	$(shell echo -m"../CMD/Debug/LED.map" >> ccsLinker.opt)
	$(shell echo --stack_size=1000 >> ccsLinker.opt)
	$(shell echo --heap_size=1000 >> ccsLinker.opt)
	$(shell echo --warn_sections >> ccsLinker.opt)
	$(shell echo -i"C:/Program Files (x86)/Texas Instruments/ccsv4/tools/compiler/c2000/lib" -i"C:/Program Files (x86)/Texas Instruments/ccsv4/tools/compiler/c2000/include" -i"C:/Users/Administrator/Desktop/LED" >> ccsLinker.opt)
	$(shell echo --reread_libs >> ccsLinker.opt)
	$(shell echo --rom_model >> ccsLinker.opt)
	$(shell echo $< >> ccsLinker.opt)
	$(shell type ccsObjs.opt >> ccsLinker.opt)
	"C:/Program Files (x86)/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" -@ccsLinker.opt -o "$@"
	@echo 'Finished building: $<'
	@echo ' '


