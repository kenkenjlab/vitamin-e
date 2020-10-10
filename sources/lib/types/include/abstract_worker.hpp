#ifndef ABSTRACT_WORKER_HPP
#define ABSTRACT_WORKER_HPP

#include "definition.hpp"

class AbstractWorker
{
public:
	AbstractWorker();
	~AbstractWorker();

	bool initialize();
	
	bool finalize();

	virtual bool clear() = 0;

	inline bool isInitialized() const { return initialized_; }

private:
	virtual bool initializeImpl_() = 0;

	virtual bool finalizeImpl_() = 0;

	bool initialized_;
};

#endif