BINS := /tmp/test_c_str

all: $(BINS)
clean:
	rm -f $(BINS)

/tmp/%: %.cpp
	g++ $< -o $@

