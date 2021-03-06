#include <memory>

namespace dronecore {
namespace backend {

class DroneCoreBackend
{
public:
    DroneCoreBackend();
    ~DroneCoreBackend();
    DroneCoreBackend(DroneCoreBackend &&) = delete;
    DroneCoreBackend &operator=(DroneCoreBackend &&) = delete;

    bool run(const int mavlink_listen_port = 14540);

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace backend
} // namespace dronecore
