BR2_DIR   ?= /home/minimoog/Developer/buildroot
TOOLCHAIN := $(BR2_DIR)/output/host/bin
CROSS     := x86_64-buildroot-linux-gnu-
CC        := $(TOOLCHAIN)/$(CROSS)gcc
 
TARGET := myscratch_test
SRC    := myscratch_test.c
 
default: $(TARGET)
 
$(TARGET): $(SRC)
	$(CC) -Wall -o $(TARGET) $(SRC)
 
clean:
	rm -f $(TARGET)
 
.PHONY: default clean
 

