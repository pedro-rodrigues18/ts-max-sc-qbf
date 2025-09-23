CXX      := -g++
CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror -pthread -fopenmp
LDFLAGS  := -L$(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi120
BUILD    := ./build
OBJ_DIR  := $(BUILD)/objects
APP_DIR  := $(BUILD)
TARGET   := app
INCLUDE  := -Iinclude/ -I$(GUROBI_HOME)/include
SRC      := $(wildcard src/*.cpp) $(wildcard src/tabu-search/*.cpp) $(wildcard src/sc-qbf/*.cpp)

OBJECTS  := $(SRC:%.cpp=$(OBJ_DIR)/%.o) 

all: build $(APP_DIR)/$(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
	
$(APP_DIR)/$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $(APP_DIR)/$(TARGET) $(OBJECTS) $(LDFLAGS)
	
.PHONY:  all build clean debug release run

build:
	@mkdir -p $(APP_DIR)
	@mkdir -p $(OBJ_DIR)
	
debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O3
release: all

clean:
	-@rm -rvf $(OBJ_DIR)/*
	-@rm -rvf $(APP_DIR)/*
	
run:
	./$(BUILD)/$(TARGET)
