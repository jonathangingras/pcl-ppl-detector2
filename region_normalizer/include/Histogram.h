#include <stdlib.h>
#include <set>
#include <vector>
#include <algorithm>
#include <stdexcept>

template<typename T>
class Bin {
protected:
	size_t count;
	T min;
	T max;
public:
	inline Bin(T _min, T _max): min(_min), max(_max) {}
	inline ~Bin() {}
	inline T averageValue() const {
		return (max - min) * 0.5;
	}
	inline size_t size() const {
		return count;
	}
	inline bool handles(T _val) const {
		return (min <= _val && _val <= max);
	}
	inline void increase() {
		++count;
	}
	inline bool operator< (const Bin& _bin) {
		return (averageValue() < _bin.averageValue());
	}
};

//note: Type T must override operator < or be a premitive type (float type or integer type)
template <typename T>
class Histogram {
private:
	std::vector<Bin<T> > bins;

	typename std::vector<Bin<T> >::iterator getHighestBin() {
		typename std::vector<Bin<T> >::iterator highestBin;
		size_t maxSamples = 0;
		for(typename std::vector<Bin<T> >::iterator bin = bins.begin(); bin != bins.end(); ++bin) {
			if(maxSamples < bin->size()) {
				maxSamples = bin->size();
				highestBin = bin;
			}
		}
		return highestBin;
	}

public:
	inline Histogram(size_t _numberBins, T _min, T _max) {
		T range = _max - _min;
		T binSize = range / (float)_numberBins;

		T lowerBound = _min;
		for(size_t i = 0; i < _numberBins; ++i) {
			bins.push_back( Bin<T>(lowerBound, lowerBound + binSize) );
			lowerBound += binSize;

			ROS_ERROR_STREAM("lowerBound " << i << ":" << lowerBound);
		}
	}
	inline ~Histogram() {}
	T getPeek() {
		//for(int i = 0; i < 10; ++i){ROS_ERROR_STREAM("bin " << i << ":" << bins[i].size());}
		return getHighestBin()->averageValue();
	}
	void push(T value) {
		for(typename std::vector<Bin<T> >::iterator bin = bins.begin(); bin != bins.end(); ++bin) {
			if(bin->handles(value)) {
				bin->increase();
				break;
			}
		}
	}
	size_t getNumberBins() const {
		return bins.size();
	}
	size_t size() const {
		size_t nb_samples = 0;
		for(typename std::vector<Bin<T> >::const_iterator bin = bins.begin(); bin != bins.end(); ++bin) {
			nb_samples += bin->size();
		}
		return nb_samples;
	}
};

//#include "impl/Histogram.hpp"