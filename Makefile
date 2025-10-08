CC = clang
CFLAGS = -Wall -Wextra -std=c11 -Iinclude -g
LDFLAGS = -Llib -framework CoreVideo -framework IOKit -framework Cocoa -framework GLUT -framework OpenGL -lraylib

OBJ_DIR = obj
CORE_DIR = core
SCENARIOS_DIR = scenarios

SCENARIO_SRCS = $(wildcard $(SCENARIOS_DIR)/*.c)
SCENARIO_OBJS = $(patsubst $(SCENARIOS_DIR)/%.c,$(OBJ_DIR)/%.o,$(SCENARIO_SRCS))
SCENARIOS = $(notdir $(basename $(SCENARIO_SRCS)))

CORE_SRCS = $(wildcard $(CORE_DIR)/*.c)
CORE_OBJS = $(patsubst $(CORE_DIR)/%.c,$(OBJ_DIR)/%.o,$(CORE_SRCS))

all: $(CORE_OBJS) $(SCENARIOS)

$(SCENARIOS): $(SCENARIOS_DIR)/*.c
	$(CC) $(CFLAGS) -c $(SCENARIOS_DIR)/$@.c -o $(OBJ_DIR)/$@.o
	$(CC) $(CORE_OBJS) $(OBJ_DIR)/$@.o -o $@ $(LDFLAGS)

# Compile source files to object files
$(OBJ_DIR)/%.o: $(CORE_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $^ -o $@

# Create directories if they don't exist
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(SCENARIOS)

.PHONY: all clean

