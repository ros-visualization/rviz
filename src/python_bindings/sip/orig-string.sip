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


void __str__() /NonLazy/;
%MethodCode
	const char* s;
        std::string* ptr;


        if (sipParseArgs(&sipArgsParsed,sipArgs,"J1",sipClass_std_string, &ptr))
        {
	        s = ptr->c_str();
	        /* Python API reference, P40 */
	        return PyString_FromString(s);
        }
%End


void __getitem__() /NonLazy/;
%MethodCode
        std::string* ptr;
        int i;
        /* J1: an object of class type. Expects corresping class object, and 
               a pointer-to-pointer for the return result. 
           i: an integer. Expects a pointer-to-integer
        */              
 
        if (sipParseArgs(&sipArgsParsed,sipArgs,"J1i",sipClass_std_string, &ptr, &i))
        {

            if (i >= ptr->length())
            {
                    /* Python API Reference, Chapter 4 */
                    PyErr_SetString ( PyExc_IndexError  ,"string index out of range" );
                    return NULL;
            }
            else
                    /* 
                       Python API reference, P23 
                       Extending and Embedding the Python Interpreter, 1.3-7 P8-11
                     
                     */
                    return Py_BuildValue("c", ptr->at(i));
        }
%End

}; /* class string */
}; /*namespace std */
