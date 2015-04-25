# Top level makefile:
#  
CXX		:= c++
CXXFLAGS 	:=  -lpthread -lrt
INCLUDES	:=  -I /usr/include/boost/
OBJECTS		:=
LIBS      := 
 
all: main.cpp
	$(CXX) main.cpp -o hitl  -std=c++11  ${INCLUDES} ${CXXFLAGS} ${LIBS} -lboost_thread -lboost_system
	
clean:
	rm pokus
