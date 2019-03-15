#include "ParameterStorage.hpp"

ParameterStorage::ParameterStorage() {
    loadFile();
}

void ParameterStorage::reloadFile() {
    std::lock_guard<std::mutex> b_lock(param_bool_mutex);
    std::lock_guard<std::mutex> i_lock(param_int_mutex);
    std::lock_guard<std::mutex> d_lock(param_double_mutex);
    std::lock_guard<std::mutex> s_lock(param_string_mutex);
    std::lock_guard<std::mutex> is_lock(param_ints_mutex);
    std::lock_guard<std::mutex> ds_lock(param_doubles_mutex);

    param_bool.clear();
    param_int.clear();
    param_double.clear();
    param_string.clear();
    param_ints.clear();
    param_doubles.clear();

    loadFile();
}

void ParameterStorage::loadFile() {
    std::cout << "Trying to load file..." << std::endl;
    YAML::Node file = YAML::LoadFile("test.yaml");
    std::cout << "\tGot file" << std::endl;

    YAML::Node params = file["parameters"];
    YAML::Node params_bool = params["bool"];
    YAML::Node params_int = params["int"];
    YAML::Node params_double = params["double"];
    YAML::Node params_string = params["string"];
    YAML::Node params_ints = params["ints"];
    YAML::Node params_doubles = params["doubles"];
    
    assert(params_bool.IsMap());
    assert(params_int.IsMap());
    assert(params_double.IsMap());
    assert(params_string.IsMap());
    assert(params_ints.IsMap());
    assert(params_doubles.IsMap());

    for (YAML::const_iterator it=params_bool.begin();it!=params_bool.end();++it) {
        set_parameter_bool(it->first.as<std::string>(), it->second.as<bool>());
    }
    for (YAML::const_iterator it=params_int.begin();it!=params_int.end();++it) {
        set_parameter_int(it->first.as<std::string>(), it->second.as<int32_t>());
    }
    for (YAML::const_iterator it=params_double.begin();it!=params_double.end();++it) {
        set_parameter_double(it->first.as<std::string>(), it->second.as<double>());
    }
    for (YAML::const_iterator it=params_string.begin();it!=params_string.end();++it) {
        set_parameter_string(it->first.as<std::string>(), it->second.as<std::string>());
    }
    for (YAML::const_iterator outer_it=params_ints.begin();outer_it!=params_ints.end();++outer_it) {
        std::vector<int32_t> ints;
        assert(outer_it->second.IsSequence());
        for (YAML::const_iterator inner_it=outer_it->second.begin();inner_it!=outer_it->second.end();++inner_it) {
            ints.push_back(inner_it->as<int32_t>());
        }
        set_parameter_ints(outer_it->first.as<std::string>(), ints);
        // std::cout << "Loaded " << outer_it->first.as<std::string>() << " with values " << std::endl;
        // for (auto val : ints) {
        //     std::cout << "\t" << val << std::endl;
        // }
    }
    for (YAML::const_iterator outer_it=params_doubles.begin();outer_it!=params_doubles.end();++outer_it) {
        std::vector<double> doubles;
        assert(outer_it->second.IsSequence());
        for (YAML::const_iterator inner_it=outer_it->second.begin();inner_it!=outer_it->second.end();++inner_it) {
            doubles.push_back(inner_it->as<double>());
        }
        set_parameter_doubles(outer_it->first.as<std::string>(), doubles);
    }

    std::cout << "\tFile loaded" << std::endl;
}

void ParameterStorage::storeFile() {
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "parameters";
    out << YAML::Value << YAML::BeginMap;
    
    out << YAML::Key << "bool";
    out << YAML::Value << YAML::BeginMap;
    for (auto const& key : list_bool()) {
        bool value;
        get_parameter_bool(key, value);
        out << YAML::Key << key;
        out << YAML::Value << value;
    }
    out << YAML::EndMap;

    out << YAML::Key << "int";
    out << YAML::Value << YAML::BeginMap;
    for (auto const& key : list_int()) {
        int32_t value;
        get_parameter_int(key, value);
        out << YAML::Key << key;
        out << YAML::Value << value;
    }
    out << YAML::EndMap;

    out << YAML::Key << "double";
    out << YAML::Value << YAML::BeginMap;
    for (auto const& key : list_double()) {
        double value;
        get_parameter_double(key, value);
        out << YAML::Key << key;
        out << YAML::Value << YAML::DoublePrecision(PRECISION) << value;
    }
    out << YAML::EndMap;

    out << YAML::Key << "string";
    out << YAML::Value << YAML::BeginMap;
    for (auto const& key : list_string()) {
        std::string value;
        get_parameter_string(key, value);
        out << YAML::Key << key;
        out << YAML::Value << YAML::DoubleQuoted << value;
    }
    out << YAML::EndMap;

    out << YAML::Key << "ints";
    out << YAML::Value << YAML::BeginMap;
    for (auto const& key : list_ints()) {
        std::vector<int32_t> value;
        get_parameter_ints(key, value);
        out << YAML::Key << key;
        out << YAML::Value << value;
    }
    out << YAML::EndMap;
    
    out << YAML::Key << "doubles";
    out << YAML::Value << YAML::BeginMap;
    for (auto const& key : list_doubles()) {
        std::vector<double> value;
        get_parameter_doubles(key, value);
        out << YAML::Key << key;
        out << YAML::Value << YAML::DoublePrecision(PRECISION) << value;
    }
    out << YAML::EndMap;

    out << YAML::EndMap << YAML::EndMap;

    std::cout << out.c_str() << std::endl;

    std::ofstream file;
    file.open("test_out.yaml", std::ofstream::out | std::ofstream::trunc);
    file << out.c_str() << std::endl;
    file.close();
}

void ParameterStorage::set_parameter_bool(std::string name, bool value) {
    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
    if (param_bool.find(name) != param_bool.end()) {
        param_bool[name] = value;
    }
    else {
        param_bool.emplace(name, value);
    }
}
void ParameterStorage::set_parameter_int(std::string name, int32_t value) {
    std::lock_guard<std::mutex> u_lock(param_int_mutex);
    if (param_int.find(name) != param_int.end()) {
        param_int[name] = value;
    }
    else {
        param_int.emplace(name, value);
    }
}
void ParameterStorage::set_parameter_double(std::string name, double value) {
    std::lock_guard<std::mutex> u_lock(param_double_mutex);
    if (param_double.find(name) != param_double.end()) {
        param_double[name] = value;
    }
    else {
        param_double.emplace(name, value);
    }
}
void ParameterStorage::set_parameter_string(std::string name, std::string value) {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    if (param_string.find(name) != param_string.end()) {
        param_string[name] = value;
    }
    else {
        param_string.emplace(name, value);
    }
}
void ParameterStorage::set_parameter_string(std::string name, const char* value) {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    if (param_string.find(name) != param_string.end()) {
        param_string[name] = value;
    }
    else {
        param_string.emplace(name, value);
    }
}
void ParameterStorage::set_parameter_ints(std::string name, std::vector<int32_t> value) {
    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
    if (param_ints.find(name) != param_ints.end()) {
        param_ints[name] = value;
    }
    else {
        param_ints.emplace(name, value);
    }
}
void ParameterStorage::set_parameter_doubles(std::string name, std::vector<double> value) {
    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
    if (param_doubles.find(name) != param_doubles.end()) {
        param_doubles[name] = value;
    }
    else {
        param_doubles.emplace(name, value);
    }
}

bool ParameterStorage::get_parameter_bool(std::string name, bool& value) {
    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
    if (param_bool.find(name) != param_bool.end()) {
        value = param_bool[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_int(std::string name, int32_t& value) {
    std::lock_guard<std::mutex> u_lock(param_int_mutex);
    if (param_int.find(name) != param_int.end()) {
        value = param_int[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_double(std::string name, double& value) {
    std::lock_guard<std::mutex> u_lock(param_double_mutex);
    if (param_double.find(name) != param_double.end()) {
        value = param_double[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_string(std::string name, std::string& value) {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    if (param_string.find(name) != param_string.end()) {
        value = param_string[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_ints(std::string name, std::vector<int32_t>& value) {
    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
    if (param_ints.find(name) != param_ints.end()) {
        value = param_ints[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_doubles(std::string name, std::vector<double>& value) {
    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
    if (param_doubles.find(name) != param_doubles.end()) {
        value = param_doubles[name];
        return true;
    }
    return false;
}

std::vector<std::string> ParameterStorage::list_bool() {
    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_bool) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_int() {
    std::lock_guard<std::mutex> u_lock(param_int_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_int) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_double() {
    std::lock_guard<std::mutex> u_lock(param_double_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_double) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_string() {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_string) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_ints() {
    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_ints) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_doubles() {
    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_doubles) {
        param_names.push_back(entry.first);
    }
    return param_names;
}