#include "operator_counting.h"

#include "partition.h"

#include <algorithm>
#include <cctype>
#include <istream>
#include <iterator>
#include <ostream>
#include <stdexcept>

using namespace std;

namespace ccbp {
namespace {
bool is_space(char c) {
    return isspace(static_cast<unsigned char>(c));
}

bool is_digit(char c) {
    return isdigit(static_cast<unsigned char>(c));
}

string strip(const string &s) {
    size_t begin = 0;
    size_t end = s.size();
    while (begin < end && is_space(s[begin]))
        ++begin;
    while (end > begin && is_space(s[end - 1]))
        --end;
    return s.substr(begin, end - begin);
}

// s[pos..] matches \s*-?\d+ up to the end of s.
bool parse_rhs(const string &s, size_t pos, long long &rhs) {
    while (pos < s.size() && is_space(s[pos]))
        ++pos;
    size_t start = pos;
    if (pos < s.size() && s[pos] == '-')
        ++pos;
    size_t digits = pos;
    while (pos < s.size() && is_digit(s[pos]))
        ++pos;
    if (pos == digits || pos != s.size())
        return false;
    rhs = stoll(s.substr(start));
    return true;
}

/*
  Python: re.match(r"c\d+:\s*(.*?)\s*>=\s*(-?\d+)$", candidate). The lazy
  group ends at the first ">=" that is followed by the right-hand side only.
*/
bool match_constraint(const string &candidate, string &terms, long long &rhs) {
    size_t pos = 0;
    if (candidate.empty() || candidate[pos] != 'c')
        return false;
    ++pos;
    size_t digits = pos;
    while (pos < candidate.size() && is_digit(candidate[pos]))
        ++pos;
    if (pos == digits || pos >= candidate.size() || candidate[pos] != ':')
        return false;
    ++pos;
    for (size_t ge = candidate.find(">=", pos); ge != string::npos; ge = candidate.find(">=", ge + 1)) {
        if (parse_rhs(candidate, ge + 2, rhs)) {
            terms = candidate.substr(pos, ge - pos);
            return true;
        }
    }
    return false;
}

/*
  Python: re.finditer(r"([+-]?)\s*(\d*)\s*\*?\s*x(\d+)", terms), tried at each
  position from left to right. Returns the end of the match, or npos.
*/
size_t match_term(const string &terms, size_t pos, char &sign, string &coefficient, long long &index) {
    sign = '+';
    if (pos < terms.size() && (terms[pos] == '+' || terms[pos] == '-'))
        sign = terms[pos++];
    while (pos < terms.size() && is_space(terms[pos]))
        ++pos;
    size_t coefficient_start = pos;
    while (pos < terms.size() && is_digit(terms[pos]))
        ++pos;
    coefficient = terms.substr(coefficient_start, pos - coefficient_start);
    while (pos < terms.size() && is_space(terms[pos]))
        ++pos;
    if (pos < terms.size() && terms[pos] == '*')
        ++pos;
    while (pos < terms.size() && is_space(terms[pos]))
        ++pos;
    if (pos >= terms.size() || terms[pos] != 'x')
        return string::npos;
    ++pos;
    size_t index_start = pos;
    while (pos < terms.size() && is_digit(terms[pos]))
        ++pos;
    if (pos == index_start)
        return string::npos;
    index = stoll(terms.substr(index_start, pos - index_start));
    return pos;
}

vector<int> sorted_unique(vector<int> values) {
    sort(values.begin(), values.end());
    values.erase(unique(values.begin(), values.end()), values.end());
    return values;
}
}

void OperatorCountingLP::sort_constraints_by_actions() {
    stable_sort(constraints.begin(), constraints.end(),
                [](const OpCountConstraint &a, const OpCountConstraint &b) {
                    return a.pos_actions.size() + a.neg_actions.size() <
                           b.pos_actions.size() + b.neg_actions.size();
                });
}

void OperatorCountingLP::keep_n_constraints(int n) {
    if (n >= 0 && static_cast<size_t>(n) < constraints.size())
        constraints.resize(n);
}

OperatorCountingLP read_operator_counting_lp(istream &in, ostream &log) {
    OperatorCountingLP lp;
    string content((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());

    // Python: re.search(r"Subject To\s*(.*?)\s*End", content, re.DOTALL)
    size_t subject_to = content.find("Subject To");
    if (subject_to == string::npos)
        return lp;
    size_t block_start = subject_to + string("Subject To").size();
    size_t block_end = content.find("End", block_start);
    if (block_end == string::npos)
        return lp;
    string block = content.substr(block_start, block_end - block_start);

    string carry;
    int num_skipped = 0;
    size_t line_start = 0;
    while (line_start <= block.size()) {
        size_t line_end = block.find_first_of("\r\n", line_start);
        if (line_end == string::npos)
            line_end = block.size();
        string line = strip(block.substr(line_start, line_end - line_start));
        if (line_end < block.size() && block[line_end] == '\r' &&
            line_end + 1 < block.size() && block[line_end + 1] == '\n')
            ++line_end;
        line_start = line_end + 1;
        if (line.empty())
            continue;

        string candidate = carry.empty() ? line : strip(carry + " " + line);
        string terms;
        long long rhs;
        if (!match_constraint(candidate, terms, rhs)) {
            carry = candidate;
            continue;
        }

        OpCountConstraint constraint;
        constraint.rhs = static_cast<int>(rhs);
        size_t pos = 0;
        while (pos < terms.size()) {
            char sign;
            string coefficient;
            long long index;
            size_t end = match_term(terms, pos, sign, coefficient, index);
            if (end == string::npos) {
                ++pos;
                continue;
            }
            if (!coefficient.empty() && coefficient != "1")
                lp.all_unit_coefficients = false;
            int action = static_cast<int>(index - 1);
            if (sign == '-')
                constraint.neg_actions.push_back(action);
            else
                constraint.pos_actions.push_back(action);
            pos = end;
        }
        constraint.pos_actions = sorted_unique(move(constraint.pos_actions));
        constraint.neg_actions = sorted_unique(move(constraint.neg_actions));

        if (!constraint.pos_actions.empty() || !constraint.neg_actions.empty()) {
            if (constraint.neg_actions.empty() && constraint.rhs <= 0)
                ++num_skipped;
            else
                lp.constraints.push_back(move(constraint));
        }
        carry.clear();
    }
    if (num_skipped > 0)
        log << "Skipped " << num_skipped << " always satisfied operator-counting constraint(s)" << '\n';
    return lp;
}

Partition build_operator_counting_partition(
    const OperatorCountingLP &lp, const vector<int> &original_to_label, int num_labels) {
    Partition groups(num_labels);
    int num_original = original_to_label.size();
    for (const OpCountConstraint &constraint : lp.constraints) {
        for (const vector<int> *actions : {&constraint.pos_actions, &constraint.neg_actions}) {
            vector<int> labels;
            for (int action : *actions) {
                if (0 <= action && action < num_original)
                    labels.push_back(original_to_label[action]);
            }
            labels = sorted_unique(move(labels));
            if (!labels.empty())
                groups.refine(labels);
        }
    }
    return groups;
}
}
