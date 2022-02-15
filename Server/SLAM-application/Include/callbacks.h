#pragma once

#include <map>
#include <functional>
#include <any>

namespace NTNU::utility
{
template<typename Key, typename FunParam = std::any>
class callbacks
{
	using Fun = std::function<void(FunParam)>;

public:
	void enable_callback(Key key, Fun fun) { cbs_.insert_or_assign(key, fun); };

	void call_callback(Key key, FunParam param) {
		if (auto cb = cbs_.find(key); cb != cbs_.end())
			cb->second(param);
	};

	// Class should be inherited, not instantiated on its own.
	virtual ~callbacks() = 0 {};

private:
	std::map<Key, Fun> cbs_;
};

}

