LIBRARY := libnd.a

CXXFLAGS += -O2 -Wall -Wextra

SUFFIX := cpp

SOURCES := Scan.cpp calcul.cpp nd.cpp geometria.cpp
			
OBJECTS := $(patsubst %.$(SUFFIX),%.o,$(SOURCES))

DEPENDENCIES := $(patsubst %.$(SUFFIX),%.d,$(SOURCES))
			
#rules
all: $(LIBRARY)

$(LIBRARY): $(OBJECTS)
	$(AR) rs $(LIBRARY) $(OBJECTS)
	
clean:
	$(RM) $(OBJECTS) $(LIBRARY) $(DEPENDENCIES)
	
.PHONY: clean all

#dependencies
-include $(DEPENDENCIES)

#main rule
%.o: %.$(SUFFIX)
	$(CXX) $(CXXFLAGS) -DNDEBUG -MMD -MT "$*.d $*.o" -c $< -o $@

