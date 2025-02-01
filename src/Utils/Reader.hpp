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

#ifndef Antipatrea__Reader_HPP_
#define Antipatrea__Reader_HPP_

#include <istream>
#include "Utils/Definitions.hpp"

namespace Antipatrea {
    /**
     *@brief Interface for reading from an input stream.
     */
    class Reader {
    public:
        Reader(void) {
        }

        virtual ~Reader(void) {
        }

        /**
         *@brief Read a double value from the input stream.
         *
         *@remarks
         * - ReadDouble will read not only double values but also inf,
         *   which will be assigned the value INFINITY.
           */
        static Status ReadDouble(std::istream &in, double &val);


        /**
         *@brief Open a file for reading, read the content, and then close the file.
         *
         *@remarks
         * - The file is opened in ios_base::in mode by default.
         * - The function Read is used to read the content from the file.
         */
        virtual Status ReadFromFile(const char fname[],
                                    std::ios_base::openmode mode = std::ios_base::in);

        /**
         *@brief Read the content from the input stream.
         */
        virtual Status Read(std::istream &in) = 0;
    };

}

#endif



