#ifndef COMMANDLINEPARSER_COMMANDLINEPARSER_H
#define COMMANDLINEPARSER_COMMANDLINEPARSER_H

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <sstream>
#include <vector>

namespace commonOptions {

template<typename T>
struct OptionDescription {
	std::string optionName;
	std::string description;
	T defaultValue;
	T value;
};

template<typename T>
using OptionDescriptionMap = std::map<std::string, std::shared_ptr<OptionDescription<T>>>;

enum class ParaType { None, One, Multi };
class AllOptions {
public:
	template<typename T>
	static OptionDescriptionMap<T>& getOptionDescriptionMap() {
		static OptionDescriptionMap<T> map;
		return map;
	}
	static std::map<std::string, std::function<void(std::string const&)>>& parseMap() {
		static std::map<std::string, std::function<void(std::string const&)>> map;
		return map;
	}
	static std::map<std::string, std::function<void()>>& preParseMap() {
		static std::map<std::string, std::function<void()>> map;
		return map;
	}

	static std::map<std::string, std::function<void()>>& postParseMap() {
		static std::map<std::string, std::function<void()>> map;
		return map;
	}
	static std::map<std::string, std::function<void()>>& printMap() {
		static std::map<std::string, std::function<void()>> map;
		return map;
	}
	static std::map<std::string, ParaType>& parseParaMap() {
		static std::map<std::string, ParaType> map;
		return map;
	}


};

/**
 * This type must be convertable by using stringstream
 */
template<typename T>
class Option {
private:
	std::shared_ptr<OptionDescription<T>> value;
public:
	Option(std::string const& _name, T const& _default, std::string const& _description)
		: Option(_name, _default, _description, [](T const&){}) {
	}

	Option(std::string _name, T const& _default, std::string const& _description, std::function<void(T const&)> _func) {
		std::transform(_name.begin(), _name.end(), _name.begin(), ::tolower);

		auto& map = AllOptions::getOptionDescriptionMap<T>();
		if (map[_name] == nullptr) {
			map[_name] = std::make_shared<OptionDescription<T>>();
			map[_name]->optionName   = _name;
			map[_name]->description  = _description;
			map[_name]->defaultValue = _default;
			map[_name]->value        = _default;
		}
		value = map[_name];
		AllOptions::parseMap()[_name] = [&](std::string const& _name) {
			std::stringstream ss;
			ss<<_name;
			ss>>value->value;
		};
		AllOptions::printMap()[_name] = [=]() {
			std::stringstream ss;
			ss<<"--"<<value->optionName<<" "<<value->defaultValue;
			while(ss.str().length() < 32) {
				ss<<" ";
			}
			ss<<value->description;
			std::cout<<ss.str()<<std::endl;
		};
		AllOptions::parseParaMap()[_name] = ParaType::One;
		AllOptions::preParseMap()[_name] = []() {};
		AllOptions::postParseMap()[_name] = [this, _func]() { _func(value->value); };
	}

	T const* operator->() const {
		return &value->value;
	}
	T const& operator*() const {
		return value->value;
	}
};
template<typename T>
class Option<std::vector<T>> {
private:
	std::shared_ptr<OptionDescription<std::vector<T>>> value;
public:
	Option(std::string const& _name, std::vector<T> const& _default, std::string const& _description)
		: Option(_name, _default, _description, [](std::vector<T> const&){}) {
	}

	Option(std::string _name, std::vector<T> const& _default, std::string const& _description, std::function<void(std::vector<T> const&)> _func) {
		std::transform(_name.begin(), _name.end(), _name.begin(), ::tolower);

		auto& map = AllOptions::getOptionDescriptionMap<std::vector<T>>();
		if (map[_name] == nullptr) {
			map[_name] = std::make_shared<OptionDescription<std::vector<T>>>();
			map[_name]->optionName   = _name;
			map[_name]->description  = _description;
			map[_name]->defaultValue = _default;
			map[_name]->value        = _default;
		}
		value = map[_name];
		AllOptions::parseMap()[_name] = [&](std::string const& _name) {
			std::stringstream ss;
			ss<<_name;
			T t;
			ss>>t;
			value->value.push_back(t);
		};
		AllOptions::printMap()[_name] = [=]() {
			std::stringstream ss;
			ss<<"--"<<value->optionName<<" { ";
			for (auto const& v : value->defaultValue) {
				ss << v << ", ";
			}
			ss<<"}";
			while(ss.str().length() < 32) {
				ss<<" ";
			}
			ss<<value->description;
			std::cout<<ss.str()<<std::endl;
		};
		AllOptions::parseParaMap()[_name] = ParaType::Multi;
		AllOptions::preParseMap()[_name] = [&]() { value->value.clear(); };
		AllOptions::postParseMap()[_name] = [this, _func]() { _func(value->value); };
	}

	std::vector<T> const* operator->() const {
		return &value->value;
	}
	std::vector<T> const& operator*() const {
		return value->value;
	}
};
template<typename T>
class Option<std::set<T>> {
private:
	std::shared_ptr<OptionDescription<std::set<T>>> value;
public:
	Option(std::string const& _name, std::set<T> const& _default, std::string const& _description)
		: Option(_name, _default, _description, [](std::set<T> const&){}) {
	}

	Option(std::string _name, std::set<T> const& _default, std::string const& _description, std::function<void(std::set<T> const&)> _func) {
		std::transform(_name.begin(), _name.end(), _name.begin(), ::tolower);

		auto& map = AllOptions::getOptionDescriptionMap<std::set<T>>();
		if (map[_name] == nullptr) {
			map[_name] = std::make_shared<OptionDescription<std::set<T>>>();
			map[_name]->optionName   = _name;
			map[_name]->description  = _description;
			map[_name]->defaultValue = _default;
			map[_name]->value        = _default;
		}
		value = map[_name];
		AllOptions::parseMap()[_name] = [&](std::string const& _name) {
			std::stringstream ss;
			ss<<_name;
			T t;
			ss>>t;
			value->value.push_back(t);
		};
		AllOptions::printMap()[_name] = [=]() {
			std::stringstream ss;
			ss<<"--"<<value->optionName<<" { ";
			for (auto const& v : value->defaultValue) {
				ss << v << ", ";
			}
			ss<<"}";
			while(ss.str().length() < 32) {
				ss<<" ";
			}
			ss<<value->description;
			std::cout<<ss.str()<<std::endl;
		};
		AllOptions::parseParaMap()[_name] = ParaType::Multi;
		AllOptions::preParseMap()[_name] = [&]() { value->value.clear(); };
		AllOptions::postParseMap()[_name] = [this, _func]() { _func(value->value); };
	}

	std::set<T> const* operator->() const {
		return &value->value;
	}
	std::set<T> const& operator*() const {
		return value->value;
	}
};
inline bool& hasError() {
	static bool error{false};
	return error;
}


class Switch : public Option<bool> {
public:
	Switch(std::string const& _name, std::string const& _description)
		: Switch(_name, _description, []() {}) {
	}
	Switch(std::string _name, std::string const& _description, std::function<void()> _func)
		: Option(_name, false, _description) {
		std::transform(_name.begin(), _name.end(), _name.begin(), ::tolower);

		AllOptions::parseParaMap()[_name] = ParaType::None;
		AllOptions::preParseMap()[_name] = []() {};
		AllOptions::postParseMap()[_name] = _func;
	}


	operator bool() {
		return **this;
	}
};

inline void print() {
	for (auto f : AllOptions::printMap()) {
		f.second();
	}
}

/**
 * This is a very simple command line parser
 * It can parse stuff like
 * --option
 * --option value
 * --opiton=value
 *
 *  it will ignore everything else on the command line. This is needed because boost::
 *  program_options is handling unknown options very badly
 */
inline bool parse(int argc, char const* const* argv) {
	std::map<std::string, std::string> options;

	for (int i(0); i<argc; ++i) {
		std::string arg = argv[i];
		if (0 == arg.compare(0, 2, "--")) {
			arg = arg.substr(2); // cut of first two symbols
			std::transform(arg.begin(), arg.end(), arg.begin(), ::tolower);

			size_t equalSignPos = arg.find("=");
			if (equalSignPos != std::string::npos) {
				std::string key   = arg.substr(0, equalSignPos);
				std::string value = arg.substr(equalSignPos+1);

				if (AllOptions::parseParaMap().find(key) == AllOptions::parseParaMap().end()) {
					hasError() = true;
					continue;
				}
				AllOptions::preParseMap()[key]();
				AllOptions::parseMap()[key](value);
				AllOptions::postParseMap()[key]();
			} else if (AllOptions::parseParaMap().find(arg) == AllOptions::parseParaMap().end()) {
				hasError() = true;
			} else if (AllOptions::parseParaMap().at(arg) == ParaType::Multi) {
				std::string key   = arg.substr(0, equalSignPos);
				AllOptions::preParseMap()[key]();
				while (i+1 < argc && std::string(argv[i+1]).compare(0, 2, "--") != 0) {
					std::string value = argv[i+1];
					AllOptions::parseMap()[key](value);
					++i;
				}
				AllOptions::postParseMap()[key]();
			} else if (i+1 < argc
					   && std::string(argv[i+1]).compare(0, 2, "--") != 0) {
				std::string key   = arg;

				if (AllOptions::parseParaMap().at(key) == ParaType::One) {
					std::string value = argv[i+1];
					AllOptions::preParseMap()[key]();
					AllOptions::parseMap()[key](value);
					AllOptions::postParseMap()[key]();
					++i;
				} else {
					AllOptions::parseMap()[key]("1");
				}
			} else {
				AllOptions::preParseMap()[arg]();
				AllOptions::parseMap()[arg]("1");
				AllOptions::postParseMap()[arg]();

			}
		}
	}
	return not hasError();
}

}

#endif
