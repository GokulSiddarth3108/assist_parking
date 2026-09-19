# Artifact name
ARTIFACT = assist_parking

# Build architecture/variant string
PLATFORM ?= aarch64le

# Build profile
BUILD_PROFILE ?= debug

CONFIG_NAME ?= $(PLATFORM)-$(BUILD_PROFILE)

OUTPUT_DIR = build/$(CONFIG_NAME)

# Main application target
TARGET = $(OUTPUT_DIR)/$(ARTIFACT)

# Pass 1 test target
TEST_TARGET = $(OUTPUT_DIR)/test_fault_detector


# ============================================================
# Compiler definitions
# ============================================================

CC = qcc -Vgcc_nto$(PLATFORM)

CXX = q++ -Vgcc_nto$(PLATFORM)_cxx

LD = $(CC)


# ============================================================
# User defined include/preprocessor flags and libraries
# ============================================================

LIBS += -L/root/hardware-component-samples/common/rpi_gpio/build/aarch64le-debug -lrpi_gpio -lm


# ============================================================
# Compiler flags for build profiles
# ============================================================

CCFLAGS_release += -O2

CCFLAGS_debug += -g -O0 -fno-builtin

CCFLAGS_coverage += -g -O0 -ftest-coverage -fprofile-arcs

LDFLAGS_coverage += -ftest-coverage -fprofile-arcs

CCFLAGS_profile += -g -O0 -finstrument-functions

LIBS_profile += -lprofilingS


# ============================================================
# Generic compiler flags
# ============================================================

CCFLAGS_all += -Wall -fmessage-length=0

CCFLAGS_all += $(CCFLAGS_$(BUILD_PROFILE))

LDFLAGS_all += $(LDFLAGS_$(BUILD_PROFILE))

LIBS_all += $(LIBS_$(BUILD_PROFILE))


# ============================================================
# Dependency generation
# ============================================================

DEPS = -Wp,-MMD,$(@:%.o=%.d),-MT,$@


# ============================================================
# Main application source files
#
# IMPORTANT:
# test_fault_detector.c is NOT included here because it
# contains its own main().
# ============================================================

SRCS = src/main.c \
       src/ultrasonic_driver.c \
       src/imu_driver.c \
       src/fault_detector.c \
       src/sensor_buffer.c \
       src/sensor_manager.c \
       src/fusion.c
       
# ============================================================
# Main application object files
# ============================================================

OBJS = $(addprefix $(OUTPUT_DIR)/,$(addsuffix .o,$(basename $(SRCS))))


# ============================================================
# Pass 1 test source/object
# ============================================================

TEST_SRC = src/test_fault_detector.c

TEST_OBJ = $(OUTPUT_DIR)/$(basename $(TEST_SRC)).o


# ============================================================
# Compiling rule
# ============================================================

$(OUTPUT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(DEPS) -o $@ $(INCLUDES) $(CCFLAGS_all) $(CCFLAGS) $<


# ============================================================
# Main application linking rule
# ============================================================

$(TARGET): $(OBJS)
	$(LD) -o $(TARGET) $(LDFLAGS_all) $(LDFLAGS) $(OBJS) $(LIBS_all) $(LIBS)


# ============================================================
# Pass 1 test linking rule
#
# test_fault_detector has its own main().
# It only needs the detector implementation.
# ============================================================

$(TEST_TARGET): $(TEST_OBJ) $(OUTPUT_DIR)/src/fault_detector.o
	$(LD) -o $(TEST_TARGET) $(LDFLAGS_all) $(LDFLAGS) \
		$(TEST_OBJ) \
		$(OUTPUT_DIR)/src/fault_detector.o \
		$(LIBS_all) $(LIBS)


# ============================================================
# Default build
#
# Builds BOTH executables:
#
#   assist_parking
#   test_fault_detector
# ============================================================

all: $(TARGET) 

# ============================================================
# Clean
# ============================================================

clean:
	rm -fr $(OUTPUT_DIR)


# ============================================================
# Rebuild
# ============================================================

rebuild: clean all


# ============================================================
# Dependency inclusion
# ============================================================

-include $(OBJS:%.o=%.d)

-include $(TEST_OBJ:%.o=%.d)