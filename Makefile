mh400e_gearbox.so: \
		mh400e_common.h \
		mh400e_gearbox.comp \
		mh400e_gears.h \
		mh400e_gears.c \
		mh400e_twitch.h \
		mh400e_twitch.c \
		mh400e_util.h \
		mh400e_util.c
	@halcompile --compile mh400e_gearbox.comp

mh400e_gearbox_sim.so: \
		mh400e_gearbox_sim.comp \
		mh400e_common.h \
		mh400e_gears.h \
		mh400e_gears.c \
		mh400e_util.h \
		mh400e_util.c
	@halcompile --compile mh400e_gearbox_sim.comp

gearbox: mh400e_gearbox.so

sim: mh400e_gearbox_sim.so

all: gearbox sim

install: all
	@halcompile --install mh400e_gearbox.comp

install-sim: install sim
	@halcompile --install mh400e_gearbox_sim.comp

run: install-sim
	@halrun -f mh400e_gearbox_sim.hal &
	@echo Launched halrun.

clean:
	@rm -f mh400e_gearbox.so
	@rm -f mh400e_gearbox_sim.so
