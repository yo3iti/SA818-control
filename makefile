# ===============================================
# Makefile for SA818 Control Tool (sa818ctl)
# ===============================================

# Compiler and flags
CC       := gcc
CFLAGS   := -Wall -Wextra -O2
LDFLAGS  :=
TARGET   := sa818ctl
SRC      := sa818ctl.c
PREFIX   := /usr/bin

# Default language if not provided
LANG ?= en

# ===============================================
# Build rules
# ===============================================

# Define preprocessor macro depending on chosen language
ifeq ($(LANG),ro)
    CFLAGS += -DLANG_RO
    MSG_LANG = "Romanian"
else
    MSG_LANG = "English"
endif

all: $(TARGET)

# Debug build
debug: CFLAGS += -g -O0
debug: clean all

# Build the binary
$(TARGET): $(SRC)
	@echo "🔨 Building $(TARGET) [Language: $(MSG_LANG)]..."
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)
	@echo "✅ Build complete."

# Install binary to /usr/bin
install: $(TARGET)
	@echo "🚀 Installing $(TARGET) to $(PREFIX)..."
	sudo install -m 755 $(TARGET) $(PREFIX)/$(TARGET)
	@echo "✅ Installed to $(PREFIX)/$(TARGET)"

# Uninstall binary
uninstall:
	@echo "🗑️  Removing $(PREFIX)/$(TARGET)..."
	sudo rm -f $(PREFIX)/$(TARGET)
	@echo "✅ Uninstalled."

# Clean up build artifacts
clean:
	@echo "🧹 Cleaning up..."
	rm -f $(TARGET) *.o
	@echo "✅ Clean complete."

# Run directly (requires sudo for /dev/serial0)
run: $(TARGET)
	sudo ./$(TARGET)

# Help info
help:
	@echo "SA818 Control Tool Makefile"
	@echo ""
	@echo "Usage:"
	@echo "  make            - Build the program (optimized)"
	@echo "  make debug      - Build with debug info (-g)"
	@echo "  make install    - Install binary to /usr/bin"
	@echo "  make uninstall  - Remove installed binary"
	@echo "  make clean      - Remove build files"
	@echo "  make run        - Build and run (sudo)"
	@echo "  make help       - Show this help message"

.PHONY: all clean debug run help install uninstall
