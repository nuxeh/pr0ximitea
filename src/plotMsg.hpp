// PlotMsg.hpp - Lightweight, header-only JSON message builder
//
// Usage:
//   PlotMsgBuilder.init();
//   PlotMsgBuilder.add("Temp_C", 25.5);
//   Serial.print(PlotMsgBuilder.get());
//
// Relies on a standard library (like <sstream>) which is usually available 
// even on Arduino-like platforms.

#include <string>
#include <sstream>

namespace PlotMsg {

class PlotMsg {
private:
    std::stringstream ss_;
    bool first_field_ = true;

public:
    // Clears the buffer and starts a new JSON object '{'
    void init() {
        ss_.str(""); // Clear previous content
        ss_.clear(); // Clear state flags
        ss_ << "{";
        first_field_ = true;
    }

    // Template method to add a key-value pair of any numeric type (int, float, double).
    template<typename T>
    void add(const char* key, T value) {
        if (!first_field_) {
            ss_ << ", ";
        }
        
        // Format: "key": value
        // The value is printed without quotes, as it is a numeric type.
        ss_ << "\"" << key << "\": " << value;
        first_field_ = false;
    }

    // Finalizes the JSON structure '}' and returns the completed C-string.
    // NOTE: The C-string pointer is only valid until the next call to init() or get().
    const char* get() {
        if (!first_field_) {
            ss_ << "}";
        } else {
            ss_ << "{}"; // Handle case where no fields were added
        }
        
        return ss_.str().c_str();
    }
};

} // namespace PlotMsg
