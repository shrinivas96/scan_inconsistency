/*
 * Consist, a software for checking map consistency in SLAM
 * Copyright (C) 2013-2014 Mladen Mazuran and Gian Diego Tipaldi and
 * Luciano Spinello and Wolfram Burgard and Cyrill Stachniss
 *
 * This file is part of Consist.
 *
 * Consist is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Consist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Consist.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ARGUMENTPARSER_H_
#define ARGUMENTPARSER_H_

#include <vector>
#include <string>

class ArgumentParser {
public:

    ArgumentParser();
    virtual ~ArgumentParser();

    void addOption(
            const char *longname, char shortname,
            char *&destvar, const char *helpstring);
    void addOption(
            const char *longname, char shortname,
            double &destvar, const char *helpstring);
    void addOption(
            const char *longname, char shortname,
            int &destvar, const char *helpstring);
    void addOption(
            const char *longname, char shortname,
            bool &destvar, const char *helpstring);

    void addArgument(char *&destvar, const char *displayname, const char *helpstring);
    void addArgument(double &destvar, const char *displayname, const char *helpstring);
    void addArgument(int &destvar, const char *displayname, const char *helpstring);

    bool parse(int argc, char **argv);

    void printHelp();

    static bool fileExists(const char *filename);

private:
    enum ArgumentType {
        StringType, BoolType, IntType, DoubleType
    };

    struct OptionEntry {
        std::string longname;
        char shortname;
        void *destvar;
        ArgumentType type;
        std::string helpstring;
    };

    struct ArgumentEntry {
        void *destvar;
        ArgumentType type;
        std::string displayname;
        std::string helpstring;
    };

    template <typename T>
    void loadOptionValue(T arg, char *value);

    void printHelpLine(const std::string &text);
    std::string formatParameter(const std::string &text, void *argument, ArgumentType type);
    void addOption(
            const char *longname, char shortname,
            void *destvar, ArgumentType type, const char *helpstring);
    void addArgument(
            void *destvar, ArgumentType type, const char *displayname, const char *helpstring);

    std::vector<OptionEntry> _options;
    std::vector<ArgumentEntry> _arguments;
    std::string _binname;
};

#endif /* ARGUMENTPARSER_H_ */
