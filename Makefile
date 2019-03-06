build_target = NUCLEO_F401RE
build_toolchain = GCC_ARM
build_dir = BUILD
project_name = launch-firmware

outdir = out/
outname = $(board).bin
outpath = $(outdir)$(outname)

validate:
ifndef board
		$(error Board is not set to one of: bb, gs, fc, tpc)
endif

build: validate msg_downlink_generated.h msg_uplink_generated.h msg_fc_update_generated.h
	time mbed compile --target $(build_target) --toolchain $(build_toolchain) -D$(board) \
	&& mkdir -p $(outdir) && cp $(build_dir)/$(build_target)/$(build_toolchain)/$(project_name).bin $(outpath) \
	&& echo "Copied output to $(outpath)"

flash: validate
	st-flash write $(outpath) 0x8000000

%_generated.h: general/%.fbs
	general/flatc --cpp $<

build-all:
	make build board=bb
	make build board=fc
	make build board=gs
	make build board=tpc