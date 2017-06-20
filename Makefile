export PROGRAM = espway
SRC_FOLDER = src
N_PROCESSES ?= 5

all: firmware

firmware: $(SRC_FOLDER)/firmware/$(PROGRAM).bin

$(SRC_FOLDER)/firmware/$(PROGRAM).bin: $(SRC_FOLDER)/fsdata.c
	$(MAKE) -j$(N_PROCESSES) -C $(SRC_FOLDER) all

$(SRC_FOLDER)/fsdata.c: frontend
	cd frontend; npm run build
	perl scripts/makefsdata

clean:
	$(MAKE) -C $(SRC_FOLDER) clean
	rm -f $(SRC_FOLDER)/fsdata.c

