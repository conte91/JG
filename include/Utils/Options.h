#include <string>

namespace Utils{

  template<const char ... CHARS>
  struct constString {
    operator const std::string&()
    {
        static constexpr std::string str{ { CHARS... } };
        return str;
    }
  };

  template<typename T, T default_value, typename noStringType,  typename ... Others >
  class Options
  {
    
    private:
      /** In order to check type in static assert, we need to prevent this function to be instantiated also if template is not instantiated */
      template<typename K>
      bool alwaysFalse(){
        return false;
      }

      Options(){
        static_assert(alwaysFalse<noStringType>, "Only string-named options are supported" );
      }
  };

  template<const char ... NAME> 
  template<typename T, T default_value,  typename ... Others >
  class Options<T, default_value, constString<NAME...>, Others... >
  :
  Options<Others...>
  {
    static constexpr std::string OPTION_NAME =(std::string)constString<NAME...>{};

  };

#if 0
  template<class T>
    void globalOption(std::string name, T defaultValue){
    }

  template<class T>
    static Options<T> globalOptions(){
      static Options<T> options;
      return options;
    }
#endif
}
