# Top level makefile:
#  
CXX		:= c++
CXXFLAGS 	:=  -lpthread -lrt 
INCLUDES	:=  -I /usr/include/boost/ /usr/include/JSBSim/ 
OBJECTS		:=
LIBS      	:= -lboost_thread -lboost_system 
 
all: main.cpp
	$(CXX) main.cpp -o hitl  -std=c++11  ${INCLUDES} ${CXXFLAGS} ${LIBS} -llibJSBSim.so
	
clean:
	rm pokus
