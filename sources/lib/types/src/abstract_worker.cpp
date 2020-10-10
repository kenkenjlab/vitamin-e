#include "abstract_worker.hpp"

AbstractWorker::AbstractWorker()
    : initialized_(false)
{}

AbstractWorker::~AbstractWorker()
{}

bool AbstractWorker::initialize()
{
	bool ret = finalize();
	if (ret)
	{
		initialized_ = true;
		ret &= initializeImpl_();
	}
	return ret;
}

bool AbstractWorker::finalize()
{
	bool ret = true;
	if (initialized_)
	{
		ret = finalizeImpl_();
		initialized_ = false;
	}
	return ret;
}