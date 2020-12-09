// Copyright (c) 2018-2020 jsonxx - Nomango
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once
#include <cstdio>  // std::FILE
#include <type_traits>
#include <ios>

namespace jsonxx
{
    //
    // input_adapter
    //

    template <typename _CharTy>
    struct input_adapter
    {
        using char_type = _CharTy;
        using char_traits = std::char_traits<char_type>;

        virtual typename char_traits::int_type get_char() = 0;
        virtual ~input_adapter() = default;
    };

    template <typename _CharTy>
    struct file_input_adapter
        : public input_adapter<_CharTy>
    {
        using char_type = typename input_adapter<_CharTy>::char_type;
        using char_traits = typename input_adapter<_CharTy>::char_traits;

        file_input_adapter(std::FILE *file) : file(file) {}

        virtual typename char_traits::int_type get_char() override
        {
            return std::fgetc(file);
        }

    private:
        std::FILE *file;
    };

    template <typename _CharTy>
    struct stream_input_adapter
        : public input_adapter<_CharTy>
    {
        using char_type = typename input_adapter<_CharTy>::char_type;
        using char_traits = typename input_adapter<_CharTy>::char_traits;

        stream_input_adapter(std::basic_istream<char_type> &stream) : stream(stream), streambuf(*stream.rdbuf()) {}

        virtual typename char_traits::int_type get_char() override
        {
            auto ch = streambuf.sbumpc();
            if (ch == EOF)
            {
                stream.clear(stream.rdstate() | std::ios::eofbit);
            }
            return ch;
        }

        virtual ~stream_input_adapter()
        {
            stream.clear(stream.rdstate() & std::ios::eofbit);
        }

    private:
        std::basic_istream<char_type> &stream;
        std::basic_streambuf<char_type> &streambuf;
    };

    template <typename _StringTy>
    struct string_input_adapter
        : public input_adapter<typename _StringTy::value_type>
    {
        using char_type = typename input_adapter<typename _StringTy::value_type>::char_type;
        using char_traits = typename input_adapter<typename _StringTy::value_type>::char_traits;

        string_input_adapter(const _StringTy &str) : str(str), index(0) {}

        virtual typename char_traits::int_type get_char() override
        {
            if (index == str.size())
                return char_traits::eof();
            return str[index++];
        }

    private:
        const _StringTy &str;
        typename _StringTy::size_type index;
    };

    template <typename _CharTy>
    struct buffer_input_adapter
        : public input_adapter<_CharTy>
    {
        using char_type = typename input_adapter<_CharTy>::char_type;
        using char_traits = typename input_adapter<_CharTy>::char_traits;

        buffer_input_adapter(const _CharTy *str) : str(str), index(0) {}

        virtual typename char_traits::int_type get_char() override
        {
            if (str[index] == '\0')
                return char_traits::eof();
            return str[index++];
        }

    private:
        const char_type *str;
        std::size_t index;
    };

    //
    // json_lexer & json_parser
    //

    enum class token_type
    {
        uninitialized,

        literal_true,
        literal_false,
        literal_null,

        value_string,
        value_integer,
        value_float,

        begin_array,
        end_array,

        begin_object,
        end_object,

        name_separator,
        value_separator,

        parse_error,

        end_of_input
    };

    template <typename _BasicJsonTy>
    struct json_lexer
    {
        using string_type = typename _BasicJsonTy::string_type;
        using char_type = typename _BasicJsonTy::char_type;
        using integer_type = typename _BasicJsonTy::integer_type;
        using float_type = typename _BasicJsonTy::float_type;
        using boolean_type = typename _BasicJsonTy::boolean_type;
        using array_type = typename _BasicJsonTy::array_type;
        using object_type = typename _BasicJsonTy::object_type;
        using char_traits = std::char_traits<char_type>;

        json_lexer(input_adapter<char_type> *adapter) : adapter(adapter)
        {
            // read first char
            read_next();
        }

        typename char_traits::int_type read_next()
        {
            current = adapter->get_char();
            return current;
        }

        void skip_spaces()
        {
            while (current == ' ' || current == '\t' || current == '\n' || current == '\r')
            {
                read_next();
            }
        }

        token_type scan()
        {
            skip_spaces();

            token_type result = token_type::uninitialized;
            switch (current)
            {
            case '[':
                result = token_type::begin_array;
                break;
            case ']':
                result = token_type::end_array;
                break;
            case '{':
                result = token_type::begin_object;
                break;
            case '}':
                result = token_type::end_object;
                break;
            case ':':
                result = token_type::name_separator;
                break;
            case ',':
                result = token_type::value_separator;
                break;

            case 't':
                return scan_literal("true", token_type::literal_true);
            case 'f':
                return scan_literal("false", token_type::literal_false);
            case 'n':
                return scan_literal("null", token_type::literal_null);

            case '\"':
                return scan_string();

            case '-':
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                return scan_number();

            case '\0':
            case char_traits::eof():
                return token_type::end_of_input;

            // unexpected char
            default:
                return token_type::parse_error;
            }

            // skip current char
            read_next();

            return result;
        }

        token_type scan_literal(const char_type *text, token_type result)
        {
            for (std::size_t i = 0; text[i] != '\0'; ++i)
            {
                if (text[i] != char_traits::to_char_type(current))
                {
                    return token_type::parse_error;
                }
                read_next();
            }
            return result;
        }

        token_type scan_string()
        {
            if (current != '\"')
                return token_type::parse_error;

            string_buffer.clear();

            while (true)
            {
                const auto ch = read_next();
                switch (ch)
                {
                case char_traits::eof():
                {
                    // unexpected end
                    return token_type::parse_error;
                }

                case '\"':
                {
                    // skip last `\"`
                    read_next();
                    return token_type::value_string;
                }

                case 0x00:
                case 0x01:
                case 0x02:
                case 0x03:
                case 0x04:
                case 0x05:
                case 0x06:
                case 0x07:
                case 0x08:
                case 0x09:
                case 0x0A:
                case 0x0B:
                case 0x0C:
                case 0x0D:
                case 0x0E:
                case 0x0F:
                case 0x10:
                case 0x11:
                case 0x12:
                case 0x13:
                case 0x14:
                case 0x15:
                case 0x16:
                case 0x17:
                case 0x18:
                case 0x19:
                case 0x1A:
                case 0x1B:
                case 0x1C:
                case 0x1D:
                case 0x1E:
                case 0x1F:
                {
                    // invalid control character
                    return token_type::parse_error;
                }

                case '\\':
                {
                    switch (read_next())
                    {
                    case '\"':
                        string_buffer.push_back('\"');
                        break;
                    case '\\':
                        string_buffer.push_back('\\');
                        break;
                    case '/':
                        string_buffer.push_back('/');
                        break;
                    case 'b':
                        string_buffer.push_back('\b');
                        break;
                    case 'f':
                        string_buffer.push_back('\f');
                        break;
                    case 'n':
                        string_buffer.push_back('\n');
                        break;
                    case 'r':
                        string_buffer.push_back('\r');
                        break;
                    case 't':
                        string_buffer.push_back('\t');
                        break;

                    case 'u':
                    {
                        // unicode escapes
                        uint16_t byte = 0;

                        for (const auto factor : {12, 8, 4, 0})
                        {
                            const auto n = read_next();
                            if (n >= L'0' && n <= L'9')
                            {
                                byte += ((n - L'0') << factor);
                            }
                            else if (n >= L'A' && n <= L'F')
                            {
                                byte += ((n - L'A' + 10) << factor);
                            }
                            else if (n >= L'a' && n <= L'f')
                            {
                                byte += ((n - L'a' + 10) << factor);
                            }
                            else
                            {
                                // '\u' must be followed by 4 hex digits
                                return token_type::parse_error;
                            }
                        }

                        string_buffer.push_back(char_traits::to_char_type(byte));
                        break;
                    }

                    default:
                    {
                        return token_type::parse_error;
                    }
                    }
                    break;
                }

                default:
                {
                    if (ch > 0x1F && ch < 0x7F)
                    {
                        string_buffer.push_back(char_traits::to_char_type(ch));
                        break;
                    }
                    else
                    {
                        return token_type::parse_error;
                    }
                }
                }
            }
        }

        token_type scan_number()
        {
            is_negative = false;
            number_value = static_cast<float_type>(0.0);

            if (current == '-')
            {
                return scan_negative();
            }

            if (current == '0')
            {
                return scan_zero();
            }

            return scan_integer();
        }

        token_type scan_negative()
        {
            if (current == '-')
            {
                is_negative = true;
                read_next();

                return scan_integer();
            }
            return token_type::parse_error;
        }

        token_type scan_zero()
        {
            if (current == '0')
            {
                if (read_next() == '.')
                    return scan_float();
                else
                    return token_type::value_integer;
            }
            return token_type::parse_error;
        }

        token_type scan_integer()
        {
            if (std::isdigit(current))
            {
                number_value = static_cast<float_type>(current - '0');

                while (true)
                {
                    const auto ch = read_next();
                    if (ch == '.')
                        return scan_float();

                    if (ch == 'e' || ch == 'E')
                        return scan_exponent();

                    if (std::isdigit(ch))
                        number_value = number_value * 10 + (ch - '0');
                    else
                        break;
                }
                return token_type::value_integer;
            }
            return token_type::parse_error;
        }

        token_type scan_float()
        {
            if (current != '.')
                return token_type::parse_error;

            if (std::isdigit(read_next()))
            {
                float_type fraction = static_cast<float_type>(0.1);
                number_value += static_cast<float_type>(current - '0') * fraction;

                while (true)
                {
                    const auto ch = read_next();
                    if (ch == 'e' || ch == 'E')
                        return scan_exponent();

                    if (std::isdigit(ch))
                    {
                        fraction *= static_cast<float_type>(0.1);
                        number_value += static_cast<float_type>(ch - '0') * fraction;
                    }
                    else
                        break;
                }
                return token_type::value_float;
            }
            return token_type::parse_error;
        }

        token_type scan_exponent()
        {
            if (current != 'e' && current != 'E')
                return token_type::parse_error;

            // skip current char
            read_next();

            if ((std::isdigit(current) && current != '0') || (current == '-') || (current == '+'))
            {
                float_type base = 10;
                if (current == '+')
                {
                    read_next();
                }
                else if (current == '-')
                {
                    base = static_cast<float_type>(0.1);
                    read_next();
                }

                unsigned int exponent = static_cast<unsigned int>(current - '0');
                while (std::isdigit(read_next()))
                {
                    exponent = (exponent * 10) + static_cast<unsigned int>(current - '0');
                }

                float_type power = 1;
                for (; exponent; exponent >>= 1, base *= base)
                    if (exponent & 1)
                        power *= base;

                number_value *= power;
                return token_type::value_float;
            }
            return token_type::parse_error;
        }

        integer_type token_to_integer() const
        {
            integer_type integer = static_cast<integer_type>(number_value);
            return is_negative ? -integer : integer;
        }

        float_type token_to_float() const
        {
            return is_negative ? -number_value : number_value;
        }

        string_type token_to_string() const
        {
            return string_buffer;
        }

    private:
        input_adapter<char_type> *adapter;
        typename char_traits::int_type current;

        bool is_negative;
        float_type number_value;
        string_type string_buffer;
    };

    template <typename _BasicJsonTy>
    struct json_parser
    {
        using string_type = typename _BasicJsonTy::string_type;
        using char_type = typename _BasicJsonTy::char_type;
        using integer_type = typename _BasicJsonTy::integer_type;
        using float_type = typename _BasicJsonTy::float_type;
        using boolean_type = typename _BasicJsonTy::boolean_type;
        using array_type = typename _BasicJsonTy::array_type;
        using object_type = typename _BasicJsonTy::object_type;
        using char_traits = std::char_traits<char_type>;

        json_parser(input_adapter<char_type> *adapter)
            : lexer(adapter), last_token(token_type::uninitialized)
        {
        }

        void parse(_BasicJsonTy &json)
        {
            parse_value(json);

            if (get_token() != token_type::end_of_input)
                throw json_parse_error("unexpected token, expect end");
        }

    private:
        token_type get_token()
        {
            last_token = lexer.scan();
            return last_token;
        }

        void parse_value(_BasicJsonTy &json)
        {
            switch (get_token())
            {
            case token_type::literal_true:
                json = json_type::boolean;
                json.value_.data.boolean = true;
                break;

            case token_type::literal_false:
                json = json_type::boolean;
                json.value_.data.boolean = false;
                break;

            case token_type::literal_null:
                json = json_type::null;
                break;

            case token_type::value_string:
                json = lexer.token_to_string();
                break;

            case token_type::value_integer:
                json = lexer.token_to_integer();
                break;

            case token_type::value_float:
                json = lexer.token_to_float();
                break;

            case token_type::begin_array:
                json = json_type::array;
                while (true)
                {
                    json.value_.data.vector->push_back(_BasicJsonTy());
                    parse_value(json.value_.data.vector->back());

                    // read ','
                    if (get_token() != token_type::value_separator)
                        break;
                }
                if (last_token != token_type::end_array)
                    throw json_parse_error("unexpected token in array");
                break;

            case token_type::begin_object:
                json = json_type::object;
                while (true)
                {
                    if (get_token() != token_type::value_string)
                        break;
                    string_type key = lexer.token_to_string();

                    if (get_token() != token_type::name_separator)
                        break;

                    _BasicJsonTy object;
                    parse_value(object);
                    json.value_.data.object->insert(std::make_pair(key, object));

                    // read ','
                    if (get_token() != token_type::value_separator)
                        break;
                }
                if (last_token != token_type::end_object)
                    throw json_parse_error("unexpected token in object");
                break;

            default:
                // unexpected token
                throw json_parse_error("unexpected token");
                break;
            }
        }

    private:
        json_lexer<_BasicJsonTy> lexer;
        token_type last_token;
    };

} // namespace jsonxx
