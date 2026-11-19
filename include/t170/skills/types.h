#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace t170::skills {
inline constexpr int interface_version = 1;
enum class Hand : int { Left = 0, Right = 1 };
struct Hands { bool left = true, right = true; };
constexpr int index(Hand hand) { return static_cast<int>(hand); }
constexpr bool selected(Hands hands, Hand hand) { return hand == Hand::Left ? hands.left : hands.right; }
enum class Skill { TrayGrasp, TrayPlace, ConveyorGrasp, ConveyorPlace, ConveyorRetreat };
enum class Stage { Validate, Prepare, Observe, Approach, AlignDescend, Gripper, Lift, Return, Complete };
enum class Status { Running, Succeeded, Partial, NoTarget, Failed, Cancelled, Rejected };
enum class PartState { Unknown, Empty, Holding };
enum class Evidence { Unknown, CallerReported, LegacyPolicy, GripperFeedback };
enum class Error : int {
    None = 0, InvalidRequest = 1001, Busy = 1002, Cancelled = 1003,
    NoTarget = 1101, Perception = 1102, EmptyGrasp = 1201,
    Motion = 1301, Servo = 1302, Gripper = 1401, ReleaseUnconfirmed = 1402,
    Device = 1501, Exception = 1901
};
struct Metadata {
    std::string request_id; // Empty: generate a process-local correlation ID.
    std::string workcell_id; // Runtime target identity, never bound to a robot ID.
    std::string target_id;
};
struct NominalTarget {
    // Requested/calculated goal, NOT measured final TCP or final visual correction.
    std::string frame; // arm_base or upright_arm_base; metres and ZYX Euler radians.
    std::array<double,6> pose{};
    std::optional<int> tray_row_aruco, tray_column_aruco;
};
struct HandResult {
    bool requested = false;
    bool started = false;
    bool action_completed = false; // Requested primitive; grasp/place independent of retreat.
    bool retreat_completed = false;
    PartState part = PartState::Unknown;
    Evidence evidence = Evidence::Unknown;
    Stage stage = Stage::Validate;
    Error error = Error::None;
    std::optional<Stage> failure_stage;
    std::optional<int> native_code; // Absent means not reported, never invented success.
    std::optional<NominalTarget> nominal_target;
};
struct Result {
    int version = interface_version;
    Metadata metadata;
    Skill skill = Skill::TrayGrasp;
    Status status = Status::Running;
    Stage stage = Stage::Validate;
    Error error = Error::None;
    std::optional<Stage> failure_stage;
    std::optional<int> native_code;
    std::optional<int> legacy_status; // Legacy entry result (tray enum or conveyor integer), separate from SDK code.
    std::array<HandResult, 2> hands; // [left, right], NOT legacy positional r/l order.
    std::string message;
    double elapsed_s = 0;
    bool waist_adjusted = false;
    std::optional<int> available_left, available_right;
    bool ok() const { return status == Status::Succeeded; }
};
struct Progress {
    Metadata metadata;
    Skill skill = Skill::TrayGrasp;
    Stage stage = Stage::Validate;
    Hands hands;
    std::uint64_t sequence = 0;
    double elapsed_s = 0;
};
struct Control {
    // Cooperative cancellation at phase boundaries; emergency stop remains hardware-owned.
    std::function<bool()> cancelled;
    // Serialized synchronous callback. Keep it short; observer exceptions are isolated.
    std::function<void(const Progress &)> progress;
};
const char *name(Skill value);
const char *name(Stage value);
const char *name(Status value);
const char *name(PartState value);
const char *name(Evidence value);
Error from_native_code(int code, Stage stage);
std::string to_json(const Result &result);
} // namespace t170::skills
