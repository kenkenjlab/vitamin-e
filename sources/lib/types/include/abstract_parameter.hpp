#ifndef ABSTRACT_PARAMETER_HPP
#define ABSTRACT_PARAMETER_HPP

struct AbstractParameter
{
public:
	AbstractParameter() : initialized_(false) {}
	~AbstractParameter() {}

	inline bool isInitialized() const { return initialized_; }

	inline void setInitialized() { initialized_ = true; }

	virtual bool isValid() const = 0;

private:
	bool initialized_;
};

#endif