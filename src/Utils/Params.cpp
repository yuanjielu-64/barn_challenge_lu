/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */
#include "Utils/Params.hpp"
#include "Utils/Logger.hpp"
#include <iomanip>
#include <ros/package.h>
#include "Components/Constants.hpp"

namespace Antipatrea {

    Params *Params::m_singleton = new Params();

    Params::~Params(void) {
        m_map.clear();
    }

    Params::Data *Params::GetData(const char id[]) {
        auto iter = m_map.find(id);
        if (iter == m_map.end())
            return NULL;
        return iter->second;
    }

    const char *Params::GetValue(const char id[], const char notFound[]) {
        auto data = GetData(id);
        if (data == NULL || data->m_values.size() == 0)
            return notFound;
        return data->m_values[0]->c_str();
    }

    int Params::GetValuesAsDoubles(const char id[], std::vector<double> &vals, const int nrMax) {
        auto data = GetData(id);
        if (data == NULL)
            return 0;

        const int n = std::min((int) data->m_values.size(), nrMax);
        for (int i = 0; i < n; ++i)
            vals.push_back(StrToDouble(data->m_values[i]->c_str()));
        return n;
    }

    int Params::GetValuesAsInts(const char id[], std::vector<int> &vals, const int nrMax) {
        auto data = GetData(id);
        if (data == NULL)
            return 0;

        const int n = std::min((int) data->m_values.size(), nrMax);
        for (int i = 0; i < n; ++i)
            vals.push_back(StrToInt(data->m_values[i]->c_str()));
        return n;
    }

    int Params::GetValuesAsBools(const char id[], std::vector<bool> &vals, const int nrMax) {
        auto data = GetData(id);
        if (data == NULL)
            return 0;

        const int n = std::min((int) data->m_values.size(), nrMax);
        for (int i = 0; i < n; ++i)
            vals.push_back(StrToBool(data->m_values[i]->c_str()));
        return n;
    }

    int Params::GetValuesAsDoubles(const char id[], double vals[], const int nrMax) {
        auto data = GetData(id);
        if (data == NULL)
            return 0;

        const int n = std::min((int) data->m_values.size(), nrMax);
        for (int i = 0; i < n; ++i)
            vals[i] = StrToDouble(data->m_values[i]->c_str());
        return n;
    }

    int Params::GetValuesAsInts(const char id[], int vals[], const int nrMax) {
        auto data = GetData(id);
        if (data == NULL)
            return 0;

        const int n = std::min((int) data->m_values.size(), nrMax);
        for (int i = 0; i < n; ++i)
            vals[i] = StrToInt(data->m_values[i]->c_str());
        return n;
    }

    int Params::GetValuesAsBools(const char id[], bool vals[], const int nrMax) {
        auto data = GetData(id);
        if (data == NULL)
            return 0;

        const int n = std::min((int) data->m_values.size(), nrMax);
        for (int i = 0; i < n; ++i)
            vals[i] = StrToBool(data->m_values[i]->c_str());
        return n;
    }

    void Params::SetParams(const char id[], Params *const params) {
        auto iter = m_map.find(id);
        if (iter != m_map.end()) {
            Data *data = iter->second;
            if (data->m_params)
                delete data->m_params;
            data->m_params = params;
        } else {
            Data *data = new Data();
            data->m_params = params;
            m_map.insert(std::make_pair(id, data));
        }
    }

    void Params::SetValue(const char id[], const char val[]) {
        auto iter = m_map.find(id);
        if (iter != m_map.end()) {
            Data *data = iter->second;
            if (data->m_values.size() == 0)
                data->m_values.push_back(new std::string(val));
            else
                *(data->m_values[0]) = val;
        } else {
            Data *data = new Data();
            data->m_values.push_back(new std::string(val));
            m_map.insert(std::make_pair(id, data));
        }
    }

    void Params::AddValue(const char id[], const char val[]) {
        auto iter = m_map.find(id);
        if (iter != m_map.end())
            iter->second->m_values.push_back(new std::string(val));
        else {
            Data *data = new Data();

            data->m_values.push_back(new std::string(val));
            m_map.insert(std::make_pair(id, data));
        }
    }

    void Params::ProcessArgs(const int start, const int end, char **argv) {
        for (int i = start; (i + 1) <= end; i += 2) {
            if (StrSameContent(argv[i], "ParamsExtraFile"))
                ReadFromFile(argv[i + 1]);
            else
                SetValue(argv[i], argv[i + 1]);
        }
    }


    void Params::Print(std::ostream &out) const {
        static const Params *original = this;

        for (auto &iter: m_map) {
            auto data = iter.second;

            out << " " << iter.first.c_str() << " ";
            if (data->m_values.size() > 0) {
                out << "[ ";
                for (auto &val: data->m_values)
                    out << *val << " ";
                out << "] ";
            }
            if (data->m_params != NULL) {
                out << "{";
                data->m_params->Print(out);
                out << " } ";
            }
            if (original == this)
                out << std::endl;
        }
    }

    Status Params::Read(std::istream &in) {
        std::string id;
        std::string val;
        std::string comment;

        while (in >> id) {
            if (id[0] == '#') //comment
                std::getline(in, comment);
            else if (id.back() == '}')
                return STATUS_OK;
            else {
                in >> val;
                if (!in.good())
                    return STATUS_ERROR;
                if (StrSameContent(id.c_str(), "ParamsExtraFile")) {
                    Logger::m_out << "...using ParamsExtraFile " << val << std::endl;
                    std::string package_path = ros::package::getPath(Constants::KW_ProjectName);
                    auto file = package_path + "/" + val;
                    ReadFromFile(file.c_str());
                } else if (val.length() > 0 && val[0] == '[') {
                    if (val.length() > 1)
                        AddValue(id.c_str(), &(val.c_str()[1]));

                    while ((in >> val)) {
                        if (val.back() == ']') {
                            val.pop_back();
                            if (!val.empty())
                                AddValue(id.c_str(), val.c_str());
                            break;
                        } else
                            AddValue(id.c_str(), val.c_str());
                    }
                } else if (val.length() > 0 && val[0] == '{') {
                    Params *params = new Params();
                    params->Read(in);
                    SetParams(id.c_str(), params);
                } else
                    SetValue(id.c_str(), val.c_str());
            }
        }
        return STATUS_OK;
    }
}
