include ../../Makefile.include

# Specify used paths
override CFLAGS += -I$(DIR_CV)
override CFLAGS += -I$(DIR_LIB)
override CFLAGS += -I$(DIR_MODULES)

executable := skyseg.x

OBJECTS =  	standalone_skysegment.o \
		$(DIR_LIB)/v4l/video.o \
		$(DIR_LIB)/udp/socket.o \
		$(DIR_CV)/encoding/jpeg.o \
		$(DIR_CV)/trig.o \
		$(DIR_MODULES)/ObstacleAvoidSkySegmentation/obstacleavoidskysegmentation_code.o \
		$(DIR_CV)/skysegmentation/skysegmentation.o

$(executable): $(OBJECTS)

upload:
	sb2 make -C ./ all && $(DRONE_TOOL) upload_paparazzi ./$(executable) vision

all: $(executable)

%.o:: %.c
	$(QUIET_CC)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(bins):
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.x::
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.a::
	$(QUIET_LINK)$(AR) rcs $@ $^

clean:
	echo $(PROCESSOR)
	$(QUIET_CLEAN)$(RM) -v $(targets) $(bins) *.o *.d $(OBJECTS)

-include *.d tidsp/*.d
