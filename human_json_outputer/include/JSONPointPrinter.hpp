#include <time.h>
#include <ostream>
#include <fstream>
#include <sstream>

class JSONPrinter {
private:
	std::string elements_name;

	unsigned int nb_elements;
	unsigned int nb_samples;
	
	std::ostream& os;
	std::ostringstream oss;

	inline void finishJSON() {
		os << std::endl << "] }";
	}

public:
	inline JSONPrinter(std::string _elements_name, std::ostream& _os): elements_name(_elements_name), os(_os), nb_elements(0), nb_samples(0) {
		os << "{ \"samples\": [ " << std::endl;
	}

	virtual ~JSONPrinter() {
		finishJSON();
	}

	template<typename X, typename Y, typename P>
	inline void addElement(X x, Y y, P p) {
		if(nb_elements > 0) oss << ", ";
		oss << "{ \"x\": " << x << ", \"y\": " << y << ", \"probability\": " << p << " }";
		++nb_elements;
	}

	inline void printSample() {
		if(nb_samples > 0) os << ',' << std::endl;
		os << "{ \"" << elements_name << "\": [ " << oss.str() << " ], \"nb_" << elements_name << "\": " << nb_elements << ", \"time_t\": " << time(NULL) << " }";
		nb_elements = 0;
		oss.str("");
		++nb_samples;
	}
};