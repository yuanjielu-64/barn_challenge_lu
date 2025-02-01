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

#ifndef Antipatrea__Params_HPP_
#define Antipatrea__Params_HPP_

#include "Utils/Reader.hpp"
#include "Utils/Writer.hpp"
#include "Utils/Misc.hpp"
#include <cstdlib>
#include <string>
#include <climits>
#include <cmath>
#include <unordered_map>

namespace Antipatrea {
/**
     *@brief Class to facilitate setting of parameter values.
     *
     *@remarks
     * - A parameter is defined by a keyword and the data associated with that keyword.
     * - The data could be a single value, a list of values, or a group of parameters (pointer to an instance of Params).
     * - The data is retrieved by using the keyword as the index.
     * - After retrieving the data, the calling function can enumerate the list of values or access the group of parameters.
     * - The values can be converted to strings, integers, doubles, or booleans.
     * - The parameters can also be read from an input stream and written to an output stream.
     *   The format is as follows:
     *    - <Entries> := <Entry> | <Entry><Whitespace><Entries>
     *    - <Entry>   := <Comment> | <ParamName><Data>
     *    - <Comment> := any line where the first character is #
     *    - <ParamName> := string (no spaces or other whitespaces)
     *    - <Data> := <SingleValue> | <ListOfValues> | <Group>
     *    - <SingleValue> := string 
     *    - <ListOfValues> := [<Whitespace><Values><Whitespace>]
     *    - <Values> := <SingleValue> | <SingleValue> <Whitespace> <Values>
     *    - <Group>  := {<Whitespace><Entries><Whitespace>}
     *    - <Whitespace> := any white space character, e.g, empty space, tab, newline.
     *    - An example is provided below:<br> 
          @verbatim
          NrPoints 10 
          Dims [32 64]
          PRM { NrNeighbors 10 Grid { Dims [10 20] Min [-10 10] Max [-10 10] } }   
          @endverbatim
     */
    class Params : public Reader,
                   public Writer {
    public:
        Params(void) : Reader(),
                       Writer() {
        }

        virtual ~Params(void);

        struct Data {
            Data(void) : m_params(NULL) {
            }

            virtual ~Data(void) {
                if (m_params)
                    delete m_params;
                DeleteItems<std::string *>(m_values);
            }

            std::vector<std::string *> m_values;
            Params *m_params;
        };

        virtual bool HasParam(const char id[]) {
            return m_map.find(id) != m_map.end();
        }

        virtual Data *GetData(const char id[]);

        virtual const char *GetValue(const char id[], const char notFound[] = NULL);

        virtual double GetValueAsDouble(const char id[], const double notFound = INFINITY) {
            const char *str = GetValue(id);
            if (str == NULL)
                return notFound;
            return StrToDouble(str);
        }

        virtual int GetValueAsInt(const char id[], const int notFound = INT_MAX) {
            const char *str = GetValue(id);
            if (str == NULL)
                return notFound;
            return StrToInt(str);
        }

        virtual bool GetValueAsBool(const char id[], const bool notFound = false) {
            const char *str = GetValue(id);
            if (str == NULL)
                return notFound;
            return StrToBool(str);
        }

        virtual int GetValuesAsDoubles(const char id[], std::vector<double> &vals, const int nrMax = INT_MAX);

        virtual int GetValuesAsInts(const char id[], std::vector<int> &vals, const int nrMax = INT_MAX);

        virtual int GetValuesAsBools(const char id[], std::vector<bool> &vals, const int nrMax = INT_MAX);

        virtual int GetValuesAsDoubles(const char id[], double vals[], const int nrMax);

        virtual int GetValuesAsInts(const char id[], int vals[], const int nrMax);

        virtual int GetValuesAsBools(const char id[], bool vals[], const int nrMax);

        virtual void ProcessArgs(const int start, const int end, char **argv);


        virtual void SetValue(const char id[], const char val[]);

        virtual void AddValue(const char id[], const char val[]);

        virtual void SetParams(const char id[], Params *const params);

        virtual Status Read(std::istream &in);

        virtual void Print(std::ostream &out) const;

        static Params *GetSingleton(void) {
            return m_singleton;
        }

        virtual std::unordered_map<std::string, Data *> *GetMap(void) {
            return &m_map;
        }

    protected:
        static Params *m_singleton;

        std::unordered_map<std::string, Data *> m_map;
    };
}

#endif
