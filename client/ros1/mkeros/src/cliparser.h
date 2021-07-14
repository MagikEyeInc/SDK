/* 
 * CliParserService
 * 
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 *
 */

#ifndef _CLIPARSERSERVICE_H_
#define _CLIPARSERSERVICE_H_

#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <map>

namespace mke {
namespace ros {

// ============================================================================

class ParagraphIterator {
public:

    typedef char      value_type;
    typedef char *    pointer;
    typedef char &    reference;
    typedef std::ptrdiff_t difference_type;
    typedef std::forward_iterator_tag  iterator_category;


  ParagraphIterator()
  {}

  // ==========================================================================

  ParagraphIterator(std::ostream& os,
                    std::string line_prefix,
                    unsigned max_line_length,
                    bool pad_first_line = false)
    : os_(&os),
      line_prefix_(line_prefix),
      max_line_length_(max_line_length),
      current_line_length_(),
      active_instance_(new ParagraphIterator*(this))
  {
    if (pad_first_line)
      *os_ << line_prefix;
  }

  // ==========================================================================

  ~ParagraphIterator()
  {
    if (*active_instance_ == this)
      insert_word();
  }

  // ==========================================================================

  ParagraphIterator& operator=(char c)
  {
    *active_instance_ = this;

    if (std::isspace(c))
      {
        if (word_buffer_.size() > 0)
          {
            insert_word();
          }
      }
    else
      {
        word_buffer_.push_back(c);
      }
    return *this;
  }

  // ==========================================================================

  ParagraphIterator& operator*()
  {
    return *this;
  }

  // ==========================================================================

  ParagraphIterator& operator++()
  {
    return *this;
  }

  // ==========================================================================

  ParagraphIterator  operator++(int)
  {
    return *this;
  }

private:

  void insert_word() {
    if (word_buffer_.size() == 0)
      return;

    if (word_buffer_.size() + current_line_length_ <= max_line_length_)
      {
        write_word(word_buffer_);
      }
    else
      {
        *os_ << '\n' << line_prefix_;

        if (word_buffer_.size() <= max_line_length_)
          {
            current_line_length_ = 0;
            write_word(word_buffer_);
          }
        else
          {
            for (unsigned i(0);i<word_buffer_.size();i+=max_line_length_)
              {
                current_line_length_ = 0;
                write_word(word_buffer_.substr(i, max_line_length_));
                if (current_line_length_ == max_line_length_)
                  {
                    *os_ << '\n' << line_prefix_;
                  }
              }
          }
      }

    word_buffer_ = "";
  }

  // ==========================================================================

  void write_word(const std::string& word)
  {
    *os_ << word;
    current_line_length_ += unsigned(word.size());

    if (current_line_length_ != max_line_length_)
      {
        *os_ << ' ';
        ++current_line_length_;
      }
  }

  // ==========================================================================

  std::ostream* os_;
  std::string word_buffer_;

  std::string line_prefix_;
  unsigned max_line_length_;
  unsigned current_line_length_;

  std::shared_ptr<ParagraphIterator*> active_instance_;
};

// ============================================================================
// ============================================================================

class CliOptionBase {
public:
  std::string name_;
  std::string info_;

  // ==========================================================================

  CliOptionBase(const std::string &name, const std::string &info)
    : name_(name), info_(info)
    {}

  // ==========================================================================

  virtual ~CliOptionBase()
    {}

  // ==========================================================================

  virtual void set(const std::string &value) = 0;
  virtual std::string get(bool curval = true) = 0;
  virtual std::string getHelpLine(void) = 0;
  virtual bool isBoolean(void) = 0;
  virtual void setInitValue(void) = 0;
};

// ============================================================================
// ============================================================================

template<typename T>
class CliOption : public CliOptionBase
{
private:
  T &val_;
  T initval_;

  // ==========================================================================

public:
  CliOption(T &val, const std::string &name, const std::string &info, const T& initval)
    : CliOptionBase(name, info), val_(val), initval_(initval)
    {}

  ~CliOption()
    {}

  // ==========================================================================

  void set(const std::string &value)
    {}

  // ==========================================================================

  std::string get(bool curval)
  {
    return "N/A";
  }

  // ==========================================================================

  bool isBoolean(void)
  {
    return false;
  }

  // ==========================================================================

  std::string getHelpLine(void)
  {
    std::stringstream ss;
    std::string opt = "--" + name_ + " (" + get(false) + ")";

    ss << opt << std::setw(30 - opt.size()) << " ";
    const std::string text(info_);
    
    std::copy(text.begin(), text.end(),
              ParagraphIterator(ss, "                              ", 50));
    
    return ss.str();
  }

  // ==========================================================================

  void setInitValue(void)
  {
    val_ = initval_;
  }
};

// ============================================================================

typedef CliOption<bool> CliOptionBool;

template<>
inline bool CliOptionBool::isBoolean(void)
{
  return true;
}

// ============================================================================

template<>
inline void CliOptionBool::set(const std::string &value)
{
  std::string val = value;
  std::transform(val.begin(), val.end(), val.begin(),
      [](unsigned char c){ return std::tolower(c); });

  if (val == "")
    val_ = true;
  else if (val == "false")
    val_ = false;
  else if (val == "true")
    val_ = true;
  else
    throw std::runtime_error("Unrecognized boolean value: '" + value + "'");
}

// ============================================================================

template<>
inline std::string CliOptionBool::get(bool curval)
{
  if (curval)
    return (val_) ? "true" : "false";
  else
    return (initval_) ? "true" : "false";
}

// ============================================================================

template<>
inline void CliOption<std::string>::set(const std::string &value)
{
  val_ = value;
}

// ============================================================================

template<>
inline std::string CliOption<std::string>::get(bool curval)
{
  if (curval)
    return val_;
  else
    return initval_;
}

// ============================================================================

template<>
inline void CliOption<int>::set(const std::string &value)
{
  val_ = std::stoi(value);
}

// ============================================================================

template<>
inline std::string CliOption<int>::get(bool curval)
{
  if (curval)
    return std::to_string(val_);
  else
    return std::to_string(initval_);
}

// ============================================================================
// ============================================================================

class CliParserService {
protected:

  std::string app_name_;
  std::map<std::string, CliOptionBase*> options_map_;
  std::vector<std::unique_ptr<CliOptionBase>> options_;
  std::vector<std::string> unrecognized_params_; 

  // ==========================================================================

  void registerBln(bool &bopt, const std::string &name, const std::string &info, bool inivalue)
  {
    CliOption<bool> *opt = new CliOption<bool>(bopt, name, info, inivalue);
    options_map_.emplace(name, opt);
    options_.emplace_back(opt);
  }

  // ==========================================================================

  void registerStr(std::string &sopt, const std::string &name, const std::string &info, const std::string &inivalue)
  {
    CliOption<std::string> *opt = new CliOption<std::string>(sopt, name, info, inivalue);
    options_map_.emplace(name, opt);
    options_.emplace_back(opt);
  }

  // ==========================================================================

  void registerInt(int &iopt, const std::string &name, const std::string &info, const int inivalue)
  {
    CliOption<int> *opt = new CliOption<int>(iopt, name, info, inivalue);
    options_map_.emplace(name, opt);
    options_.emplace_back(opt);
  }

  // ==========================================================================

  void initConfig(void)
  {
    for (auto &opt: options_map_)
      opt.second->setInitValue();
  }

  // ==========================================================================

  template <class Container>
  void split(const std::string& str, Container& cont, char delim = '=')
  {
      std::size_t current, previous = 0;
      current = str.find(delim);

      while (current != std::string::npos) {
          cont.push_back(str.substr(previous, current - previous));
          previous = current + 1;
          current = str.find(delim, previous);
      }

      cont.push_back(str.substr(previous, current - previous));
  }

public:

  // ==========================================================================

  CliParserService()
    : app_name_{""}, options_map_{}, options_{}, unrecognized_params_{}
  {
  }

  // ==========================================================================

  std::string getAppName(void) const
  {
    return app_name_;
  }

  // ==========================================================================

  void parseFromCmdLine(const int ac, const char * const av[])
  {
    app_name_ = av[0];

    for (int i = 1; i < ac; i++)
      {
        std::string opt(av[i]);

        if ((opt.size() < 3) || (opt.substr(0, 2) != "--"))
          {
            // This is not a recognized parameter flag
            unrecognized_params_.push_back(opt);
            continue;
          }

        std::vector<std::string> fields;
        opt = opt.substr(2);
        split(opt, fields);

        auto it = options_map_.find(fields[0]);
        if (it == options_map_.end())
          throw std::runtime_error("Unrecognized command line option: '" + fields[0] + "'");

        if (fields.size() == 1)
          {
            if (it->second->isBoolean())
              {
                it->second->set("");
              }
            else
              {
                // Try to consume next parameter
                if (i + 1 < ac)
                  it->second->set(av[++i]);
                else
                  throw std::runtime_error("Parameter value not set for: '" + fields[0] + "'");
              }
          }
        else if (fields.size() == 2)
          it->second->set(fields[1]);
        else
          throw std::runtime_error("Cannot parse command line option: '" + opt + "'");
      }
  }

  // ==========================================================================
  
  void getCommandLineHelp(std::ostream &fout) {
    fout << std::endl;
    fout << "Command line options:" << std::endl;

    for (auto &opt : options_)
      fout << opt->getHelpLine() << std::endl;
  }
};

} // namespace util
} // namespace ros



#endif // _CLIPARSERSERVICE_H_
