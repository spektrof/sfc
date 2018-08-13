#pragma once
#include <vector>

namespace parser	//only read what we need, nothing more
{
	inline int get_char_arg(const int& argc, char** argv, char* arg, char** val)
	{
		for (int i = 0; i < argc - 1; ++i)
		{
			if (!strcmp(arg, argv[i]))
			{
				*val = argv[i + 1];
				return 1;
			}
		}
		return 0;
	}

	inline int get_char_arg(std::vector<const char*> argv, char* arg, const char** val)
	{
		for (auto& it : argv)
		{
			if (!strcmp(arg, it))
			{
				*val = *(&it + 1);
				return 1;
			}
		}
		return 0;
	}

	inline int get_int_arg(const std::vector<const char*> argv, char* arg, int& val)
	{
		for (auto& it : argv)
		{
			if (!strcmp(arg, it))
			{
				val = atoi(*(&it + 1));
				return 1;
			}
		}
		return 0;
	}

	inline int get_float_arg(const std::vector<const char*> argv, char* arg, float& val)
	{
		for (auto& it : argv)
		{
			if (!strcmp(arg, it))
			{
				val = (float)atof(*(&it + 1));
				return 1;
			}
		}
		return 0;
	}

	inline int get_option_arg(const int& argc, char** argv, char* arg)
	{
		for (int i = 0; i < argc; ++i)
		{
			if (!strcmp(arg, argv[i]))
				return 1;
		}
		return 0;
	}
}