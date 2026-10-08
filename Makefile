# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) The NL Sounds contributors
# Valve Howler -- a circuit-modelled overdrive for Linux, as an LV2 plugin.
#
#   make            plugin + interface
#   make install    installs into ~/.lv2/valvehowler.lv2/
#   make install INSTALL_DIR=/usr/local/lib/lv2
#   make test       smoke check on the built bundle
#   make clean
#
# Requires a C++17 compiler, the LV2 headers, and Cairo, Xlib and FreeType for
# the interface. Nothing is vendored.

CXX      ?= g++

# Instruction-set baseline -- generic x86-64, deliberately.
#
# Do not add -mavx2/-mfma/-march=native here. Post-Haswell code belongs only
# inside the `isa_v3` clone below, which the resolver selects at run time on
# the CPUs that have it. Emitted globally, those instructions do not make the
# plugin slower on an older CPU: they make it fail to start, and it takes the
# host down with it.
ARCH     ?=

# The second clone. The same translation unit is compiled twice -- once at the
# baseline and once here -- and `instantiate()` picks one per instance. What
# keeps the linker from folding the two copies together is the namespace, not
# the visibility.
ARCH_V3  ?= -mavx2 -mfma

# Everything without LV2_SYMBOL_EXPORT leaves the .so's dynamic table, which
# lets the compiler skip the PLT on internal calls and inline across units.
VIS      := -fvisibility=hidden -fvisibility-inlines-hidden

# Oversampler geometry. Both enter CXXFLAGS, so changing either rebuilds.
TAPS     ?=
BETA     ?=
OSCFG    := $(if $(TAPS),-DNLSC_OS_TAPS=$(TAPS)) $(if $(BETA),-DNLSC_OS_BETA=$(BETA))

# `:=`, not `?=`: with `?=` a CXXFLAGS exported in the environment would strip
# the binary of its flags in silence. `make CXXFLAGS=...` on the command line
# still wins, which is the intended escape hatch.
#
# Never -ffast-math. It breaks the (double)(float) round trips and the
# non-reassociation the model depends on, and it makes the non-finite checks
# undefined behaviour -- which would make the Newton fallback undefined too.
CXXFLAGS := -O3 $(ARCH) $(VIS) $(OSCFG) -fPIC -Wall -Wextra -Wshadow -std=c++17
LDFLAGS  := -shared
LDLIBS   := -lm

# Floating-point relaxations the plugin is built with. They are applied to the
# plugin's own translation units only.
PRODFP   := -fassociative-math -fno-signed-zeros -fno-trapping-math

# The shipped model configuration.
CASCMOD  := -DNLSC_E2_OPAMP_LIN -DNLSC_E2_CHARGE_E3=1 -DNLSC_VRA_PARAM=1
CASCFG   := $(CASCMOD) -DNLSC_CASC_KHZ=192

SRC    := src
LV2DIR := lv2
BUILD  := build
BUNDLE := $(BUILD)/valvehowler.lv2

INSTALL_DIR ?= $(HOME)/.lv2

DEPFLAGS := -MMD -MP

GUI_PKGS := cairo cairo-xlib x11 freetype2
GUI_CF   := $(shell pkg-config --cflags $(GUI_PKGS))
GUI_LIBS := $(shell pkg-config --libs $(GUI_PKGS))

# The font travels inside the bundle: the interface loads it through FreeType
# directly, bypassing fontconfig, so the panel looks the same on any machine.
# It is OFL and stays OFL; its licence travels with it.
FONTS := Doto-Round-Bold.ttf OFL-Doto.txt

# The panel's images, rendered from a 3D scene: one set per box colour.
PHOTO := $(foreach g,od9 od8,plate_$(g)_active.png plate_$(g)_bypass.png \
           knob_$(g)_drive.png knob_$(g)_level.png knob_$(g)_tone.png) treadle.png

# Rebuild when the flags change, not only when a source does.
FLAGSTAMP := $(BUILD)/.flags
$(shell mkdir -p $(BUILD); \
        printf '%s|%s|%s|%s|%s\n' "$(CXXFLAGS)" "$(LDFLAGS)" "$(LDLIBS)" "$(ARCH_V3)" "$(PRODFP)" \
        | cmp -s - $(FLAGSTAMP) 2>/dev/null \
        || printf '%s|%s|%s|%s|%s\n' "$(CXXFLAGS)" "$(LDFLAGS)" "$(LDLIBS)" "$(ARCH_V3)" "$(PRODFP)" > $(FLAGSTAMP))

.PHONY: all bundle gui install test clean
all: bundle

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/isa_base.o: $(SRC)/nls_isa_tu.cpp $(FLAGSTAMP) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) $(PRODFP) $(CASCFG) -DNLSC_ISA_NS=isa_base -DNLSC_ISA_FACTORY=make_base -c -o $@ $<

# $(ARCH_V3) goes AFTER $(CXXFLAGS) so it wins over the baseline's arch flags.
$(BUILD)/isa_v3.o: $(SRC)/nls_isa_tu.cpp $(FLAGSTAMP) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) $(PRODFP) $(ARCH_V3) $(CASCFG) -DNLSC_ISA_NS=isa_v3 -DNLSC_ISA_FACTORY=make_v3 -c -o $@ $<

# The explicit -MF matters: without it -MMD writes the .d inside the bundle,
# where $@ lives, and it would be installed next to the .so.
$(BUNDLE)/valvehowler.so: $(SRC)/nls_valvehowler.cpp \
                          $(BUILD)/isa_base.o $(BUILD)/isa_v3.o \
                          $(FLAGSTAMP) | $(BUILD)
	@mkdir -p $(BUNDLE)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -MF $(BUILD)/nls_valvehowler.d $(PRODFP) $(LDFLAGS) -o $@ $< $(BUILD)/isa_base.o $(BUILD)/isa_v3.o $(LDLIBS)

$(BUNDLE)/valvehowler_ui.so: $(SRC)/ui_x11.cpp $(FLAGSTAMP) | $(BUILD)
	@mkdir -p $(BUNDLE)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -MF $(BUILD)/ui_x11.d $(GUI_CF) -I$(SRC) $(LDFLAGS) -o $@ $< $(GUI_LIBS)

gui: $(BUNDLE)/valvehowler_ui.so

bundle: $(BUNDLE)/valvehowler.so $(BUNDLE)/valvehowler_ui.so
	@cp $(LV2DIR)/manifest.ttl $(LV2DIR)/valvehowler.ttl $(LV2DIR)/valvehowler_ui.ttl $(BUNDLE)/
	@cp $(FONTS:%=assets/fonts/%) $(BUNDLE)/
	@mkdir -p $(BUNDLE)/photo
	@cp $(PHOTO:%=gui/photo/%) $(BUNDLE)/photo/
	@echo "bundle ready: $(BUNDLE)"

# Atomic install. A plain `cp` rewrites the .so in place and can crash a host
# that has it memory-mapped. Temp directory, then rename, always.
install: bundle
	@mkdir -p $(INSTALL_DIR)
	@rm -rf $(INSTALL_DIR)/valvehowler.lv2.tmp
	@cp -r $(BUNDLE) $(INSTALL_DIR)/valvehowler.lv2.tmp
	@rm -rf $(INSTALL_DIR)/valvehowler.lv2.old
	@if [ -d $(INSTALL_DIR)/valvehowler.lv2 ]; then \
	    mv $(INSTALL_DIR)/valvehowler.lv2 $(INSTALL_DIR)/valvehowler.lv2.old; fi
	@mv $(INSTALL_DIR)/valvehowler.lv2.tmp $(INSTALL_DIR)/valvehowler.lv2
	@rm -rf $(INSTALL_DIR)/valvehowler.lv2.old
	@echo "installed in $(INSTALL_DIR)/valvehowler.lv2"

# A smoke check on the built bundle, which is the artefact that gets installed.
# It answers the two questions a build can get wrong without any error: whether
# the entry point survived `-fvisibility=hidden`, and whether every file the
# interface loads at run time is actually in the bundle.
test: bundle
	@fail=0; \
	nm -D --defined-only $(BUNDLE)/valvehowler.so | grep -q ' lv2_descriptor$$' \
	  || { echo "FAIL: valvehowler.so does not export lv2_descriptor"; fail=1; }; \
	nm -D --defined-only $(BUNDLE)/valvehowler_ui.so | grep -q ' lv2ui_descriptor$$' \
	  || { echo "FAIL: valvehowler_ui.so does not export lv2ui_descriptor"; fail=1; }; \
	for f in manifest.ttl valvehowler.ttl valvehowler_ui.ttl $(FONTS) $(PHOTO:%=photo/%); do \
	  [ -f $(BUNDLE)/$$f ] || { echo "FAIL: missing from the bundle: $$f"; fail=1; }; \
	done; \
	[ $$fail -eq 0 ] && echo "bundle OK: entry points exported, all runtime files present"; \
	exit $$fail

clean:
	rm -rf $(BUILD)

-include $(BUILD)/*.d
