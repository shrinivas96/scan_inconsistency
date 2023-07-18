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

#include "argumentparser.h"
#include "consist/foreach.h"
#include <getopt.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <fstream>

ArgumentParser::ArgumentParser()
{
}

ArgumentParser::~ArgumentParser()
{
}

void ArgumentParser::printHelpLine(const std::string &text)
{
    static const int width = 80;
    static const int spacing = 10;
    std::string spacingstr(spacing, ' ');
    std::stringstream ss(text);
    int rowcount = spacing;
    std::cout << spacingstr;
    while(ss.good()) {
        std::string word;
        ss >> word;
        if(rowcount + word.length() > width) {
            rowcount = spacing;
            std::cout << std::endl << spacingstr;
        }
        std::cout << word << " ";
        rowcount += word.length() + 1;
        if(ss.peek() == '\n') {
            std::cout << std::endl << spacingstr;
            rowcount = spacing;
        }
    }
    std::cout << std::endl;
}


std::string ArgumentParser::formatParameter(
        const std::string &text, void *argument, ArgumentType type)
{
    size_t start = 0, found;
    while((found = text.find('%', start)) != std::string::npos) {
        if(found + 1 < text.size()) {
            if(text.at(found + 1) == '%') {
                start = found + 2;
            } else {
                int result;
                char *formatted;
                if(type == IntType) {
                    result = asprintf(&formatted, text.c_str(), *static_cast<int *>(argument));
                } else if(type == DoubleType) {
                    result = asprintf(&formatted, text.c_str(), *static_cast<double *>(argument));
                } else if(type == IntType) {
                    result = asprintf(&formatted, text.c_str(), *static_cast<char **>(argument));
                } else {
                    result = asprintf(&formatted, text.c_str(), *static_cast<bool *>(argument));
                }

                if(result == -1) {
                    return text;
                } else {
                    std::string retformatted(formatted);
                    free(formatted);
                    return retformatted;
                }
            }
        } else {
            return text;
        }
    }
    return text;
}

void ArgumentParser::printHelp()
{
    std::cout << "Usage: " << _binname << " [options] ";
    fforeach(ArgumentEntry &arg, _arguments) {
        std::cout << arg.displayname << " ";
    }
    std::cout << std::endl << std::endl;
    if(!_arguments.empty()) {
        std::cout << "Arguments:" << std::endl;
        fforeach(ArgumentEntry &arg, _arguments) {
            std::cout << "  " << arg.displayname << std::endl;
            printHelpLine(arg.helpstring);
        }
    }
    if(!_options.empty()) {
        std::cout << "Options:" << std::endl;
        fforeach(OptionEntry &arg, _options) {
            bool nolong = arg.longname.empty(), noshort = !arg.shortname;
            std::string longfmt = "--" + arg.longname;
            std::string shortfmt = std::string("-") + arg.shortname;

            if(arg.type != BoolType) {
                std::string str = arg.longname;
                if(nolong) str = "VALUE";
                std::transform(str.begin(), str.end(),str.begin(), toupper);
                longfmt += "=" + str;
                shortfmt += " " + str;
            }

            if(nolong) {
                std::cout << "  " << shortfmt << std::endl;
            } else if(noshort) {
                std::cout << "  " << longfmt << std::endl;
            } else {
                std::cout << "  " << shortfmt << ", " << longfmt << std::endl;
            }

            printHelpLine(formatParameter(arg.helpstring, arg.destvar, arg.type));
        }
    }
}

void ArgumentParser::addOption(
        const char *longname, char shortname, double &destvar, const char *helpstring)
{
    addOption(longname, shortname, static_cast<void *>(&destvar), DoubleType, helpstring);
}

void ArgumentParser::addOption(
        const char *longname, char shortname, int &destvar, const char *helpstring)
{
    addOption(longname, shortname, static_cast<void *>(&destvar), IntType, helpstring);
}

void ArgumentParser::addOption(
        const char *longname, char shortname, char *&destvar, const char *helpstring)
{
    addOption(longname, shortname, static_cast<void *>(&destvar), StringType, helpstring);
}

void ArgumentParser::addOption(
        const char *longname, char shortname, bool &destvar, const char *helpstring)
{
    addOption(longname, shortname, static_cast<void *>(&destvar), BoolType, helpstring);
}

void ArgumentParser::addOption(
        const char *longname, char shortname,
        void *destvar, ArgumentType type, const char *helpstring)
{
    OptionEntry entry = {
        longname, shortname, destvar, type, helpstring
    };
    _options.push_back(entry);
}

void ArgumentParser::addArgument(
        char *&destvar, const char *displayname, const char *helpstring)
{
    addArgument(static_cast<void *>(&destvar), StringType, displayname, helpstring);
}

void ArgumentParser::addArgument(
        int &destvar, const char *displayname, const char *helpstring)
{
    addArgument(static_cast<void *>(&destvar), DoubleType, displayname, helpstring);
}


void ArgumentParser::addArgument(
        double &destvar, const char *displayname, const char *helpstring)
{
    addArgument(static_cast<void *>(&destvar), IntType, displayname, helpstring);
}


void ArgumentParser::addArgument(
        void *destvar, ArgumentType type, const char *displayname, const char *helpstring)
{
    ArgumentEntry entry = {
        destvar, type, displayname, helpstring
    };
    _arguments.push_back(entry);
}

bool ArgumentParser::parse(int argc, char **argv)
{
    struct option longOptions[_options.size() + 1];
    std::string shortOptions = "";
    int i = 0, c;

    _binname = argv[0];

    fforeach(OptionEntry &arg, _options) {
        if(!arg.longname.empty()) {
            struct option opt = { arg.longname.c_str(), no_argument, 0, 256 + i };

            if(arg.type != BoolType) {
                opt.has_arg = required_argument;
            }

            longOptions[i++] = opt;
        }

        if(arg.shortname) {
            shortOptions += arg.shortname;
            if(arg.type != BoolType) {
                shortOptions += ':';
            }
        }
    }
    struct option nullopt = { NULL, 0, 0, 0 };
    longOptions[i] = nullopt;

    while ((c = getopt_long(argc, argv, shortOptions.c_str(), longOptions, NULL)) != -1) {
        if(c > 255) {
            loadOptionValue(_options[c - 256], optarg);
        } else {
            bool found = false;
            fforeach(OptionEntry &arg, _options) {
                if(c == arg.shortname) {
                    loadOptionValue(arg, optarg);
                    found = true;
                    break;
                }
            }
            if(!found) {
                return false;
            }
        }
    }

    int idx = 0;
    while(idx < _arguments.size() && optind < argc) {
        loadOptionValue(_arguments[idx], argv[optind]);
        idx++;
        optind++;
    }
    return true;
}

template <typename T>
void ArgumentParser::loadOptionValue(T arg, char *value)
{
    switch(arg.type) {
    case StringType:
        *static_cast<char ** >(arg.destvar) = strdup(value);
        break;
    case IntType:
        *static_cast<int *   >(arg.destvar) = atoi(value);
        break;
    case DoubleType:
        *static_cast<double *>(arg.destvar) = atof(value);
        break;
    case BoolType:
        *static_cast<bool *  >(arg.destvar) = true;
        break;
    }
}

bool ArgumentParser::fileExists(const char *filename)
{
    std::ifstream fin(filename, std::ios::in);

    if(fin.is_open()) {
        fin.close();
        return true;
    } else {
        return false;
    }

}
