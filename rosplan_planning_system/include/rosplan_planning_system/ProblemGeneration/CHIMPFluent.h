#ifndef CHIMP_FLUENT_HPP
#define CHIMP_FLUENT_HPP

#include <string>
#include <vector>

namespace KCL_rosplan
{

class CHIMPFluent
{

  protected:
    static int nextId;

  public:
    enum FluentType
    {
        STATE,
        TASK
    };

    CHIMPFluent(FluentType type, std::string name, const std::vector<std::string> &arguments);
    ~CHIMPFluent();

    int id_;
    FluentType type_;
    std::string name_;
    std::vector<std::string> arguments_;

    std::string chimpFormatRepr() const;

    friend std::ostream& operator<<(std::ostream &os, const CHIMPFluent &fluent);
};

} // namespace

#endif // CHIMP_FLUENT_HPP