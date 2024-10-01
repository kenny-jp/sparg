/*
 * Copyright (c) 2024 kenny-jp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef KENNY_SPARG_HPP
#define KENNY_SPARG_HPP

#if __cplusplus < 202002L
# error("sparg requires C++-20")
#endif

#include <algorithm>
#include <any>
#include <cassert>
#include <charconv>
#include <concepts>
#include <format>
#include <functional>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace sparg {

using namespace std::literals;

namespace detail {


template<class T>
concept IsArithmetic =
    std::integral<T> || std::floating_point<T>;


template<class T>
concept IsString =
    std::same_as<T, std::string> ||
    std::same_as<T, const char*> ||
    std::is_constructible_v<std::string, T>;


template<class T>
concept IsStringConstructible =
    std::is_constructible_v<T, std::string> ||
    std::is_constructible_v<T, const char*>;


template<class T>
concept IsContainer = requires(T t){
    typename T::iterator;
    typename T::size_type;
    typename T::value_type;
    { t.begin() } -> std::same_as<typename T::iterator>;
    { t.end()   } -> std::same_as<typename T::iterator>;
    { t.size()  } -> std::same_as<typename T::size_type>;
};


template<class T, class... Args>
concept IsSequenceContainer = IsContainer<T> &&
    requires(T t, typename T::iterator it, Args&&... args) {
        {t.emplace(it, std::forward<Args>(args)...)} -> std::same_as<typename T::iterator>;
    };


template<class T, class... Args>
concept IsAssociativeContainer = IsContainer<T> &&
    requires(T t, typename T::key_type key, Args&&... args){
        typename T::key_type;
        typename T::mapped_type;
        { t.operator[](key) } -> std::same_as<std::add_lvalue_reference_t<typename T::mapped_type>>;
        { t.emplace(std::forward<Args>(args)...) } ->
            std::same_as<std::pair<typename T::iterator, bool>>;
    };


template<class T>
concept IsArithmeticContainer =
    IsSequenceContainer<T> &&
    IsArithmetic<typename T::value_type>;


template<class T>
concept IsStringContainer =
    IsSequenceContainer<T> &&
    (IsString<typename T::value_type> || IsStringConstructible<typename T::value_type>);


template<class Action, class StorageType, class Data = std::remove_cvref_t<StorageType>>
concept IsStorageAction =
    std::invocable<
        std::add_rvalue_reference_t<Action>,
        std::add_lvalue_reference_t<std::remove_cv_t<StorageType>>,
        std::span<std::string>> ||
    requires(Action __action,
             std::add_lvalue_reference_t<std::remove_cv_t<StorageType>> __storage_type,
             Data __data) { { __action.store(__storage_type, std::forward<Data>(__data)) } -> std::same_as<void>; };


template<class StorageType, class StorageAction, class Data>
requires std::is_trivial_v<Data>
void storage_process(StorageType& __t, StorageAction&& __a, Data __d)
{
    if constexpr (std::invocable<StorageAction,
            std::add_lvalue_reference_t<StorageType>,
            std::remove_cvref_t<std::remove_pointer_t<Data>>
            >)
        __a(__t, __d);
    else
        __a.store(__t, __d);
}


template<class StorageType, class StorageAction, class Data>
requires (!std::is_trivial_v<Data>)
void storage_process(StorageType& __t, StorageAction&& __a, const Data& __d)
{
    if constexpr (std::invocable<StorageAction,
            std::add_lvalue_reference_t<StorageType>,
            std::add_const_t<std::add_lvalue_reference_t<Data>>
            >)
            __a(__t, __d);
    else
    __a.store(__t, __d);
}


class InvalidArgumentError : public std::invalid_argument
{
public:
    
    InvalidArgumentError(std::string_view argument_name, std::string_view invalid_value = {})
        : std::invalid_argument{argument_name.data()}
        , m_invalid_value{invalid_value}
    {}

    
    const char* argument() const noexcept
    { return std::invalid_argument::what(); }

    
    const char* invalid() const noexcept
    { return m_invalid_value.data(); }

private:
    std::string_view m_invalid_value;
};


std::string trim(std::string_view sv)
{
    if (sv.empty())
        return ""s;

    const char* begin = std::ranges::find_if_not(sv.cbegin(), sv.cend(), isspace);
    const char* end   = std::ranges::find_if_not(sv.crbegin(), sv.crend(), isspace).base();

    return begin >= end ? ""s : std::string{begin, end};
}

namespace storage_action {


struct Default
{
    
    template<class T, class U>
    requires IsStringConstructible<T>
    && std::is_assignable_v<T, U>
    void store(T& storage, const U& u)
    {
        storage = u;
    }

    
    template<class T>
    requires IsArithmetic<T>
    void store(T& storage, T value)
    {
        storage = value;
    }

    
    template<class T>
    requires IsArithmeticContainer<T>
    void store(T& storage, typename T::value_type value)
    {
        storage.emplace(std::end(storage), value);
    }

    
    template<class T>
    requires IsStringContainer<T>
    void store(T& storage, const typename T::value_type& value)
    {
        storage.emplace(std::end(storage), value);
    }
};


struct Add : public Default
{
    
    template<class T, class U>
    requires IsStringConstructible<T>
    && std::same_as<std::add_lvalue_reference_t<T>, decltype(std::declval<T>() += std::declval<U>())>
    void store(T& storage, const U& u)
    {
        storage += u;
    }

    
    template<class T>
    requires IsArithmetic<T>
    void store(T& storage, T value)
    {
        storage += value;
    }
};

} // namespace storage_action


using fn_get  = std::function<std::any(std::span<std::string>)>;


using fn_set  = std::function<void(std::span<std::string>, std::any&)>;


using fn_void = std::function<void(std::span<std::string>)>;

template<class T>
bool from_chars(std::string_view sv, T& value)
{
    const char* first = sv.data();
    auto[ptr, ec] = std::from_chars(first, first + sv.size(), value);

    if (ec != std::errc())
        return false;

    return true;
}

} // namespace detail

class Argument
{
    friend class ArgumentParser;

    
    template<std::size_t N, std::size_t... I>
    Argument(std::string_view prefix_chars, std::array<std::string_view, N>&& names, std::index_sequence<I...>)
        : m_all_names{names.begin(), names.end()}
        , m_used_name{m_all_names.front()}
        , m_is_present{0}
        , m_nvalues{1}
        , m_is_optional{(is_optional(prefix_chars, names[I]) || ...)}
        , m_enable_repeat{false}
        , m_multiple_values{false}
        , m_required{false}
    {
        assert(!detail::trim(m_used_name).empty() && "Argument name cannot be empty.");
        store_as<std::vector<std::string>>();
    }

public:
    
    template<std::size_t N> requires (N > 0)
    Argument(std::string_view prefix_chars, std::array<std::string_view, N>&& names)
        : Argument{prefix_chars, std::move(names), std::make_index_sequence<N>{}}
    {}

    
    Argument& enable_multiple_values()
    {
        m_multiple_values = true;
        return *this;
    }

    
    Argument& nvalues(unsigned int n)
    {
        m_nvalues = n;
        return *this;
    }

    
    Argument& required()
    {
        m_required = true;
        return *this;
    }

    
    Argument& choices(const std::vector<std::string>& choices)
    {
        m_choices = std::make_optional<std::vector<std::string>>(choices);
        return *this;
    }

    
    Argument& description(std::string desrciption)
    {
        m_help = std::move(desrciption);
        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Add>
    requires detail::IsStringConstructible<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType>
    Argument& store_as()
    {
        m_storage_action.emplace<detail::fn_get>([](std::span<std::string> sp)->StorageType{
            StorageType str;
            for (const auto& s : sp)
                StorageAction{}.store(str, s);

            return str;
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Add>
    requires detail::IsStringConstructible<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType>
    Argument& store_as(StorageType& storage, StorageAction&& action = {})
    {
        m_storage_action.emplace<detail::fn_set>([&](std::span<std::string> sp, std::any& v){
            for (const auto& s : sp)
                detail::storage_process(storage, action, s);

            v = std::make_any<std::add_lvalue_reference_t<StorageType>>(storage);
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Default>
    requires detail::IsStringContainer<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType, typename StorageType::value_type>
    Argument& store_as()
    {
        m_storage_action.emplace<detail::fn_get>([](std::span<std::string> sp)->StorageType{
            StorageType storage;
            for (const auto& s : sp)
                StorageAction{}.store(storage, s);

            return storage;
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Default>
    requires detail::IsStringContainer<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType, typename StorageType::value_type>
    Argument& store_as(StorageType& storage, StorageAction&& action = {})
    {
        m_storage_action.emplace<detail::fn_set>([&](std::span<std::string> sp, std::any& v){
            for (const auto& s : sp)
                detail::storage_process(storage, action, s);

            v = std::make_any<std::add_lvalue_reference_t<StorageType>>(storage);
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Add>
    requires detail::IsArithmetic<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType, StorageType>
    Argument& store_as()
    {
        using value_type = StorageType;

        m_storage_action.emplace<detail::fn_get>([&](std::span<std::string> sp){
            value_type total{0};
            StorageType storage;
            for (const auto& s : sp) {
                value_type value {0};

                if (!detail::from_chars(s, value))
                    throw detail::InvalidArgumentError(m_used_name, s);

                StorageAction{}.store(total, value);
            }

            storage = total;
            return storage;
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Add>
    requires detail::IsArithmetic<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType>
    Argument& store_as(StorageType& storage, StorageAction&& action = {})
    {
        using value_type = StorageType;

        m_storage_action.emplace<detail::fn_set>([&](std::span<std::string> sp, std::any& v){
            for (const auto& s : sp) {
                value_type value {0};

                if (!detail::from_chars(s, value))
                    throw detail::InvalidArgumentError(m_used_name, s);

                detail::storage_process(storage, action, value);

                v = std::make_any<std::add_lvalue_reference_t<StorageType>>(storage);
            }
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Default>
    requires detail::IsArithmeticContainer<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType, typename StorageType::value_type>
    Argument& store_as()
    {
        using value_type = typename StorageType::value_type;

        m_storage_action.emplace<detail::fn_get>([&](std::span<std::string> sp)->StorageType{
            StorageType storage;
            for (const auto& s : sp) {
                value_type value {0};

                if (!detail::from_chars(s, value))
                    throw detail::InvalidArgumentError(m_used_name, s);

                StorageAction{}.store(storage, value);
            }

            return storage;
        });

        return *this;
    }

    
    template<class StorageType, class StorageAction = detail::storage_action::Default>
    requires detail::IsArithmeticContainer<StorageType>
    && detail::IsStorageAction<StorageAction, StorageType, typename StorageType::value_type>
    Argument& store_as(StorageType& storage, StorageAction&& action = {})
    {
        using value_type = typename StorageType::value_type;

        m_storage_action.emplace<detail::fn_set>([&](std::span<std::string> sp, std::any& v){
            for (const auto& s : sp) {
                value_type value {0};

                if (!detail::from_chars(s, value))
                    throw detail::InvalidArgumentError(m_used_name, s);

                detail::storage_process(storage, action, value);

                v = std::make_any<std::add_lvalue_reference_t<StorageType>>(storage);
            }
        });

        return *this;
    }

    
    template<class Action, class... Args>
    requires std::invocable<Action, Args...>
    && std::same_as<std::invoke_result_t<Action, Args...>, void>
    Argument& store_as(Action&& action, Args&&... args)
    {
        m_storage_action.emplace<detail::fn_void>([action, args...](auto){
            action(std::forward<Args>(args)...);
        });

        return *this;
    }

private:
    
    static bool is_decimal_literal(std::string_view sv)
    {
        auto is_digit = [](char c)->bool {return c >= '0' && c <= '9';};

        auto consume_leading_digits = [is_digit](std::string_view& sv)->bool {
            if (sv.empty() || !is_digit(sv.front()))
                return false;

            const auto beg {sv.begin()};
            const auto end {sv.end()};
            sv = sv.substr(static_cast<std::size_t>(std::find_if_not(beg, end, is_digit) - beg));
            return true;
        };

        auto consume_prefix = [](std::string_view prefix_chars, std::string_view& sv) -> bool {
            if (prefix_chars.find(sv.front()) != std::string_view::npos) {
                sv.remove_prefix(1);
                return true;
            }
            return false;
        };

        if (sv.empty() || (!is_digit(sv.front()) && !consume_prefix(".", sv)))
            return false;

        if (!consume_leading_digits(sv))
            return false;

        if (sv.empty())
            return true;

        if (sv.front() == '.') {
            sv.remove_prefix(1);
            if (!consume_leading_digits(sv))
                return false;
            if (sv.empty())
                return true;
        }

        if (consume_prefix("Ee", sv)) {
            if (!is_digit(sv.front()) && !consume_prefix("+-", sv))
                return false;
            if (!consume_leading_digits(sv))
                return false;
        }

        return sv.empty();
    }

    
    static bool is_positional(std::string_view prefix_chars, std::string_view sv) noexcept
    {
        if (sv.empty() || prefix_chars.find(sv.front()) == std::string_view::npos)
            return true;

        sv.remove_prefix(1);

        return sv.empty() ? true : is_decimal_literal(sv);
    }

    
    static bool is_optional(std::string_view prefix_chars, std::string_view sv) noexcept
    {
        return !is_positional(prefix_chars, sv);
    }

    std::vector<std::string> m_all_names;
    std::string_view m_used_name;
    std::string m_help;

    std::optional<std::vector<std::string>> m_choices;

    std::vector<std::string> m_argument_value_strings;
    std::any m_value;

    std::variant<detail::fn_get, detail::fn_set, detail::fn_void> m_storage_action;

    unsigned int  m_is_present;
    unsigned int m_nvalues;

    bool m_is_optional;
    bool m_enable_repeat;
    bool m_multiple_values;
    bool m_required;
};


class ArgumentParser
{
    void throw_unknown_argument(std::string_view argument_name)
    { throw std::invalid_argument(std::format("error : the '{}' argument is unknown.", argument_name)); }

    void throw_argument_already_present(std::string_view argument_name)
    { throw std::invalid_argument(std::format("error : the argument '{}' is already present.", argument_name)); }

    void throw_argument_required(std::string_view argument_name)
    { throw std::invalid_argument(std::format("error : the argument '{}' is required.", argument_name)); }

    void throw_invalid_value(std::string_view argument_name, std::string_view invalid_value)
    { throw std::invalid_argument(std::format("error : the value '{}' for argument '{}' is invalid.", invalid_value, argument_name)); }

    void throw_invalid_value_choice(std::string_view argument_name, std::string_view invalid_value)
    { throw std::invalid_argument(std::format("error : the value '{}' for argument '{}' is not a valid choice.", invalid_value, argument_name)); }

    void throw_missing_argument_value(std::string_view argument_name, std::size_t count) {
        if (count > 0)
            throw std::invalid_argument(std::format("error : the argument {} require {} value(s).", argument_name, count));
        else
            throw std::invalid_argument(std::format("error : the argument {} require at least one value.",argument_name));}

public:
    
    ArgumentParser(std::string program_name, std::string description = {}, std::string prefix_chars = "-",
                   std::string usage = {}, std::string help = {})
        : m_program_name{std::move(program_name)}, m_description{std::move(description)}
        , m_prefix_chars{std::move(prefix_chars)}
    {
        assert(!detail::trim(m_program_name).empty() && "program_name cannot be empty.");
        assert(!detail::trim(m_prefix_chars).empty() && "prefix_chars cannot be empty.");

        if (!detail::trim(usage).empty()) {
            m_usage_string = std::make_optional(std::move(usage));
        }

        if (!detail::trim(help).empty()) {
            m_help_string = std::make_optional(std::move(help));
        }


        addArgument("--help", "-h", "-?")
                .description("Display this help message")
                .nvalues(0)
                .store_as([&]()->void{
                    std::cout << *this << std::endl;
                    exit(0);
                });
    }

    
    template<class... Args>
    Argument& addArgument(Args&&... args)
    {
        using array_sv = std::array<std::string_view, sizeof...(Args)>;
        using argument_iterator = std::list<Argument>::iterator;

        assert(!(m_argument_map.contains(args) || ...) && "argument name already exists");

        argument_iterator it =
            m_argument_positionals.emplace(std::end(m_argument_positionals),
                                           m_prefix_chars,
                                           array_sv{args...});

        if (it->m_is_optional)
            m_argument_optionals.splice(std::end(m_argument_optionals), m_argument_positionals, it);

        for (const auto& name : it->m_all_names)
            m_argument_map.emplace(name, it);

        return *it;
    }

    
    void parse(std::string_view args)
    {
        std::vector<std::string> vargs;

        if (auto targs = detail::trim(args); !targs.empty()) {
            std::istringstream iss{targs};
            while (iss >> vargs.emplace_back())
                if (iss.eof()) break;
        }

        parse(std::move(vargs));
    }

    
    void parse(int argc, char** argv)
    {
        std::vector<std::string> args;

        if (argc > 1)
            args.assign(argv + 1, argv + argc);

        parse(std::move(args));
    }

    
    void parse(std::vector<std::string>&& args)
    {
        auto start_arg = args.begin();
        auto end_arg = args.end();
        auto positional_arg_it = m_argument_positionals.begin();

        auto is_positional_argument = [&](std::string_view argument_name) ->bool {

            if (Argument::is_optional(m_prefix_chars, argument_name)) {
                return false;
            }

            return positional_arg_it != std::end(m_argument_positionals);
        };

        auto is_optional_argument = [&](std::string_view argument_name) ->bool {
            return m_argument_map.contains(argument_name) && m_argument_map[argument_name]->m_is_optional;
        };

        auto is_skip_argument = [&](std::string_view argument_name) ->bool {
            if (argument_name.empty())
                return false;

            for (char prefix : m_prefix_chars)
                if (argument_name.find_first_not_of(prefix) == std::string_view::npos)
                    return true;

            return false;
        };

        for (; start_arg < end_arg; ++start_arg) {
            std::string_view argument_name = *start_arg;

            if (is_optional_argument(argument_name))
            {
                Argument& argument = *m_argument_map[argument_name];

                if (!init_arg(argument))
                    throw_argument_already_present(argument_name);

                if (argument.m_multiple_values) {
                    auto it = process_parsing_multiple_values(++start_arg, end_arg, argument);

                    if (it == end_arg)
                        throw_missing_argument_value(argument_name, 0);

                    start_arg = it;
                } else if (argument.m_nvalues) {
                    auto it = process_parsing_multiple_values(start_arg + 1, end_arg, argument, argument.m_nvalues);

                    if (it >= end_arg)
                        throw_missing_argument_value(argument.m_used_name, argument.m_nvalues);

                    start_arg = it;
                }

                assign_value(argument);

            }
            else if (is_positional_argument(argument_name))
            {
                Argument& argument = *positional_arg_it;

                if (!init_arg(argument))
                    throw_argument_already_present(argument_name);

                if (argument.m_multiple_values) {
                    auto it = process_parsing_multiple_values(start_arg, end_arg, argument);

                    if (it == end_arg)
                        throw_missing_argument_value(argument_name, 0);

                    start_arg = it;
                } else if (argument.m_nvalues) {
                    auto it = process_parsing_multiple_values(start_arg, end_arg, argument, argument.m_nvalues);

                    if (it >= end_arg)
                        throw_missing_argument_value(argument.m_used_name, argument.m_nvalues);

                    start_arg = it;
                }

                assign_value(argument);
                positional_arg_it++;

            }
            else
            {
                if (!is_skip_argument(argument_name))
                    throw_unknown_argument(argument_name);
            }
        }

        validate_parsing();
    }

    
    template<class T>
    auto get_value(std::string_view argument_name) -> std::optional<T>
    {
        using value_t = T;

        if (!m_argument_map.contains(argument_name) && m_argument_map.at(argument_name)->m_value.has_value())
            return std::nullopt;

        Argument& argument = *m_argument_map[argument_name];

        try {
            value_t value = std::any_cast<value_t>(argument.m_value);
            return std::make_optional<value_t>(value);
        } catch (const std::bad_any_cast&) {
            return std::nullopt;
        }
    }

    
    bool operator[](std::string_view argument_name) const
    {
        if (!m_argument_map.contains(argument_name))
            return false;

        return m_argument_map.at(argument_name)->m_is_present;
    }

private:
    using string_vector_iterator = std::vector<std::string>::iterator;

    
    string_vector_iterator process_parsing_multiple_values(const string_vector_iterator& begin, const string_vector_iterator& end, Argument& argument)
    {
        auto it = get_last_positional_it(begin, end);

        if (it >= end)
            return end;

        if (argument.m_choices.has_value()) {
            auto[valid,s] = is_choice_valid(begin, it + 1, argument.m_choices.value());
            if (!valid)
                throw_invalid_value_choice(argument.m_used_name, s);
        }

        argument.m_argument_value_strings.assign(begin, it + 1);
        return it;
    }

    
    string_vector_iterator process_parsing_multiple_values(const string_vector_iterator& begin, const string_vector_iterator& end, Argument& argument, std::ptrdiff_t n)
    {
        auto it = begin + n;

        if (it - 1 >= end)
            return end;

        if (argument.m_choices.has_value()) {
            auto[valid,s] = is_choice_valid(begin, begin + n, argument.m_choices.value());
            if (!valid)
                throw_invalid_value_choice(argument.m_used_name, s);
        }

        argument.m_argument_value_strings.assign(begin, begin + n);
        return it - 1;
    }

    
    bool init_arg(Argument& argument)
    {
        if (argument.m_is_present && !argument.m_enable_repeat)
            return false;

        argument.m_is_present++;
        return true;
    }

    
    void assign_value(Argument& argument)
    {
        try {
            if (std::holds_alternative<detail::fn_get>(argument.m_storage_action)) {
                argument.m_value = std::get<detail::fn_get>(argument.m_storage_action)(argument.m_argument_value_strings);
            } else if (std::holds_alternative<detail::fn_set>(argument.m_storage_action)) {
                std::get<detail::fn_set>(argument.m_storage_action)(argument.m_argument_value_strings, argument.m_value);
            } else if (std::holds_alternative<detail::fn_void>(argument.m_storage_action)) {
                std::get<detail::fn_void>(argument.m_storage_action)(argument.m_argument_value_strings);
            }
        } catch(const detail::InvalidArgumentError& e) {
            throw_invalid_value(e.argument(), e.invalid());
        }
    }

    
    string_vector_iterator get_last_positional_it(const string_vector_iterator& begin, const string_vector_iterator& end)
    {
        auto it = std::ranges::find_if_not(begin, end, [&](std::string_view sv){
            return Argument::is_positional(m_prefix_chars, sv);
        });

        if (it == end) {
            if (Argument::is_positional(m_prefix_chars, *(end - 1))) {
                return end - 1;
            }
            return end;
        }

        return it != begin ? it - 1 : it;
    }

    
    std::tuple<bool, std::string> is_choice_valid(const string_vector_iterator& begin, const string_vector_iterator& end, const std::vector<std::string>& choices)
    {
        auto it = std::ranges::find_if_not(begin, end, [choices](const std::string& s){
            for (const auto& ch : choices) {
                if (ch == s)
                    return true;
            }
            return false;
        });

        return it != end ? std::make_tuple(false, *it) : std::make_tuple(true, *begin);
    }

    
    void validate_parsing()
    {
        for (const auto& arg : m_argument_positionals) {
            if (arg.m_required && (!arg.m_is_present || !arg.m_value.has_value()))
                throw_argument_required(arg.m_used_name);
        }

        for (const auto& arg : m_argument_optionals) {
            if (arg.m_required && (!arg.m_is_present || !arg.m_value.has_value()))
                throw_argument_required(arg.m_used_name);
        }
    }

    
    std::string get_usage_string() const
    {
        std::ostringstream os;

        if (!m_usage_string.has_value()) {
            os << "usage: " << m_program_name << " ";

            if (m_argument_optionals.size()) {
                os << "[OPTIONS] ";
                for (const auto& arg : m_argument_optionals)
                    if (arg.m_required)
                        os << arg.m_used_name << " ";
            }

            if (m_argument_positionals.size()) {
                os << "... ";
                for (const auto& arg : m_argument_positionals)
                    if (arg.m_required)
                        os << arg.m_used_name << " ";
            }
        } else {
            os << m_usage_string.value();
        }

        return os.str();
    }

    
    friend
    std::ostream& operator<<(std::ostream& os, const ArgumentParser& parser)
    {
        using iterator = std::list<Argument>::const_iterator;
        auto get_max_length_arg = [](const iterator& begin, const iterator& end)->std::size_t{
            auto max_it = std::max_element(begin, end, [](const Argument& lhs, const Argument& rhs){ return lhs.m_used_name.size() < rhs.m_used_name.size(); });
            return max_it->m_used_name.size();
        };

        auto print_arg = [&](std::string_view section_name, const std::list<Argument>& list){
            if (list.empty())
                return;

            os << std::format("{}:\n", section_name);
            const int max = static_cast<int>(get_max_length_arg(list.cbegin(), list.cend())) + 4;

            for (const Argument& arg : list)
                os << "  " << std::left << std::setw(max) << arg.m_used_name << arg.m_help
                   << (arg.m_used_name == list.back().m_used_name ? "" : "\n");
        };

        os << parser.get_usage_string() << "\n\n";

        if (!detail::trim(parser.m_description).empty())
            os << parser.m_description << "\n\n";

        if (!parser.m_help_string.has_value()) {
            print_arg("Argument(s)", parser.m_argument_positionals);
            os << "\n\n";
            print_arg("Option(s)", parser.m_argument_optionals);
        } else {
            os << parser.m_help_string.value();
        }

        os << "\n";

        return os;
    }

    std::string m_program_name;
    std::string m_description;

    std::optional<std::string> m_usage_string;
    std::optional<std::string> m_help_string;

    std::string m_prefix_chars;

    std::list<Argument> m_argument_optionals;
    std::list<Argument> m_argument_positionals;
    std::map<std::string_view, std::list<Argument>::iterator> m_argument_map;
};

} // namespace sparg

#endif // KENNY_SPARG_HPP
