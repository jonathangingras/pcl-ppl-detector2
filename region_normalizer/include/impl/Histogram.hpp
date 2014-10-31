template <typename T>
inline const T& Histogram<T>::min() const {
	return *values.begin();
}

template <typename T>
inline const T& Histogram<T>::max() const {
	return *--values.end();
}

template <typename T>
inline void Histogram<T>::push(T _element) {
	values.insert(_element);
}

template <typename T>
inline void Histogram<T>::setNumberBins(size_t _nb) {
	numberBins = _nb;
}

template <typename T>
inline size_t Histogram<T>::getNumberBins() const {
	return numberBins;
}

template <typename T>
inline size_t Histogram<T>::size() const {
	return values.size();
}

template <typename T>
std::vector<T> Histogram<T>::getLowerBounds() const {
	std::vector<T> bounds;

	T range = max() - min();
	range /= numberBins;

	for(T value = min(); value < max();) {
		bounds.push_back(value);
		value += range;
	}

	return bounds;
}

template <typename T>
std::vector<T> Histogram<T>::getUpperBounds() const {
	std::vector<T> bounds;

	T range = max() - min();
	range /= numberBins;

	for(T value = min(); value < max();) {
		value += range;
		bounds.push_back(value);
	}

	return bounds;
}

template <typename T>
inline size_t Histogram<T>::getNumberSamplesBetween(const T& _lower, const T& _upper) const {
	typename std::multiset<T>::const_iterator first = values.lower_bound(_lower);
	typename std::multiset<T>::const_iterator last = values.upper_bound(_upper);
	size_t counter = 0;
	while(++first != last) ++counter;
	return counter;
	//return std::distance(values.lower_bound(_lower), values.upper_bound(_upper));
}

template <typename T>
inline T Histogram<T>::getPeek() const {
	std::vector<size_t> numberSamplesPerBin;
	std::vector<T> lowerBounds = getLowerBounds();
	std::vector<T> upperBounds = getUpperBounds();

	if(lowerBounds.size() != upperBounds.size()) {ROS_ERROR_STREAM("NOT SAME SIZE" << "lower.size: " << lowerBounds.size() << " upper.size: " << upperBounds.size()); return 0;}

	size_t max_at_index = 0;
	for(size_t i = 0; i < numberBins; ++i) {
		numberSamplesPerBin.push_back( getNumberSamplesBetween(lowerBounds[i], upperBounds[i]) );
		if(i > 0 && numberSamplesPerBin[i - 1] < numberSamplesPerBin[i]) max_at_index = i;
	}

	size_t distanceToMiddle = numberSamplesPerBin[max_at_index] / 2;
	typename std::multiset<T>::const_iterator it = values.lower_bound(lowerBounds[max_at_index]);
	for(size_t j = 0; j < distanceToMiddle; ++j) ++it;

	return *it;
}