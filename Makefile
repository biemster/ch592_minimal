TOOLCHAIN_PREFIX := riscv64-elf

C_SRCS := \
	./main.c

S_SRCS := \
	./startup_CH592.S

OBJS := \
	$(foreach src,$(C_SRCS),$(subst ./,obj/,$(patsubst %.c,%.o,$(src)))) \
	$(foreach src,$(S_SRCS),$(subst ./,obj/,$(patsubst %.S,%.o,$(src))))

MAKEFILE_DEPS := \
	$(foreach obj,$(OBJS),$(patsubst %.o,%.d,$(obj)))


SECONDARY_FLASH := main.hex
SECONDARY_LIST := main.lst
SECONDARY_SIZE := main.siz
SECONDARY_BIN := main.bin

ARCH := rv32imac_zicsr

CFLAGS_COMMON := \
	-march=$(ARCH) \
	-mabi=ilp32 \
	-mcmodel=medany \
	-msmall-data-limit=8 \
	-mno-save-restore \
	-Os \
	-fmessage-length=0 \
	-fsigned-char \
	-ffunction-sections \
	-fdata-sections

.PHONY: all
all: main.elf secondary-outputs

.PHONY: clean
clean:
	-rm $(OBJS)
	-rm $(MAKEFILE_DEPS)
	-rm $(SECONDARY_FLASH)
	-rm $(SECONDARY_LIST)
	-rm $(SECONDARY_BIN)
	-rm main.elf
	-rm main.map
	-rm -r ./obj

.PHONY: secondary-outputs
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE) $(SECONDARY_BIN)

main.elf: $(OBJS)
	${TOOLCHAIN_PREFIX}-gcc \
		$(CFLAGS_COMMON) \
		-T "linker.ld" \
		-nostartfiles \
		-Xlinker --gc-sections \
		-Xlinker --print-memory-usage \
		-Wl,-Map,"main.map" \
		-Lobj \
		-o "main.elf" \
		$(OBJS)

%.hex: %.elf
	@ ${TOOLCHAIN_PREFIX}-objcopy -O ihex "$<"  "$@"

%.bin: %.elf
	$(TOOLCHAIN_PREFIX)-objcopy -O binary $< "$@"

%.lst: %.elf
	@ ${TOOLCHAIN_PREFIX}-objdump \
		--source \
		--all-headers \
		--demangle \
		--line-numbers \
		--wide "$<" > "$@"

%.siz: %.elf
	@ ${TOOLCHAIN_PREFIX}-size --format=berkeley "$<"

obj/%.o: ./%.c
	@ mkdir --parents $(dir $@)
	@ ${TOOLCHAIN_PREFIX}-gcc \
		$(CFLAGS_COMMON) \
		-I. \
		-std=gnu99 \
		-MMD \
		-MP \
		-MF"$(@:%.o=%.d)" \
		-MT"$(@)" \
		-c \
		-o "$@" "$<"

obj/%.o: ./%.S
	@ mkdir --parents $(dir $@)
	@ ${TOOLCHAIN_PREFIX}-gcc \
		$(CFLAGS_COMMON) \
		-x assembler \
		-MMD \
		-MP \
		-MF"$(@:%.o=%.d)" \
		-MT"$(@)" \
		-c \
		-o "$@" "$<"

flash: 
	chprog ${SECONDARY_BIN}

f: clean all flash
