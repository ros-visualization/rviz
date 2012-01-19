namespace std 
{
class string
{
%TypeHeaderCode
#include <string>
%End
public:
	string();
	string(const char*);
	string(const std::string&);

	bool empty();
	int length();
	int size();
	void resize(int, char);
	void resize(int);
	int capacity();
	void reserve(int =0);

	std::string& append(const std::string&);
	std::string& append(const std::string&, int, int);
	std::string& append(const char*);
	std::string& append(const char*, int);
	std::string& append(int, char);

	std::string& insert(int, const std::string&);
	std::string& insert(int, const std::string&, int, int);
	std::string& insert(int, const char*);
	std::string& insert(int, const char*, int);
	std::string& insert(int, int, char);

	int find(const std::string&, int = 0);
	int find(const char*, int, int);
	int find(const char*, int = 0);
	int find(char, int = 0);

	int rfind(const std::string&, int = -1);
	int rfind(const char*, int, int);
	int rfind(const char*, int = -1);
	int rfind(char, int = -1);

	int find_first_of(const std::string&, int = 0);
	int find_first_of(const char*, int, int);
	int find_first_of(const char*, int = 0);
	int find_first_of(char, int = 0);

	int find_first_not_of(const std::string&, int = 0);
	int find_first_not_of(const char*, int, int);
	int find_first_not_of(const char*, int = 0);
	int find_first_not_of(char, int = 0);

	int find_last_of(const std::string&, int = 0);
	int find_last_of(const char*, int, int);
	int find_last_of(const char*, int = 0);
	int find_last_of(char, int = 0);

	int find_last_not_of(const std::string&, int = 0);
	int find_last_not_of(const char*, int, int);
	int find_last_not_of(const char*, int = 0);
	int find_last_not_of(char, int = 0);

	std::string substr(int = 0, int = -1);


SIP_PYOBJECT __str__() /DocType="str"/;
%MethodCode
	const char* s = sipCpp->c_str();
        sipRes = PyString_FromString(s);
%End

}; /* class string */
}; /*namespace std */
