# Apollo HVLP Firmware v2 - Major UI Overhaul

## Summary of Changes

This update implements a major UI/UX redesign with new features and improved user experience.

### ✅ Features Added

1. **Job Time Tracking**
   - Timer starts when motor is activated
   - Pauses during Power Pause mode
   - Displays in Runtime UI (HH:MM:SS format)
   - Resets when motor restarts

2. **New Screen Structure**
   - Added `SCREEN_SUPPORT` - Support information page
   - Added `SCREEN_ABOUT` - Firmware version and total runtime display
   - Removed old PID tuning settings from user-accessible menus

### ✅ Behavior Changes

1. **Motor Start Behavior**
   - Motor now always starts at 0 PSI target
   - User must increase PSI using rotary encoder
   - First screen after splash is now Runtime (was Menu)

2. **Power Pause Timeout**
   - After 15 minutes in Power Pause, system automatically:
     - Exits to Menu UI
     - Stops the motor
     - Pauses job timer

### ✅ Runtime UI Redesign

**New Layout:**
- **Top 2/3 of screen**: Large Target PSI display with color indicator
  - GREEN: Within 0.5 PSI of target
  - RED: Out of range
  - YELLOW: Motor off/inactive
- **Bottom 1/3 split**:
  - Left side: Job Time (HH:MM:SS)
  - Right side: Temperature (°F)
- Removed: Motor power%, pause countdown, old debug info

### ✅ Menu UI Updates

**New Menu Options:**
1. Start Motor
2. Settings
3. SUPPORT
4. About

**Settings Submenu:**
- Power Pause Timeout (seconds)
- Power Pause Warning Sounds (ON/OFF)  
- Power Pause Warning Time (seconds)
- Exit Settings

(Designed with space for 2+ additional options)

### ✅ Overlay Improvements

**Power Pause Overlay:**
- Cleaner design with consistent theming
- Countdown timer showing time until menu timeout
- Clear instructions: "Press trigger or turn dial to resume motor"
- Double-border design

**Overtemp Overlay:**
- Prominent "OVERHEATING" warning
- "Motor is too hot, check filter" message
- Current temperature display
- Red color scheme for urgency

**Support Page:**
- Placeholder text box for support information
- "Press to return to menu" instruction
- Ready for customization

**About Page:**
- Displays "APOLLO HVLP" branding
- Firmware version number
- Total motor runtime hours
- Clean, professional layout

### ✅ Color Theme Management

**Centralized in `display_ui.h`:**
```cpp
// Background & Text
COLOR_BG, COLOR_TEXT_PRIMARY, COLOR_TEXT_SECONDARY

// Target Pressure Indicators  
COLOR_TARGET_ACTIVE (green)
COLOR_TARGET_OUTRANGE (red)
COLOR_TARGET_INACTIVE (yellow)

// Status Colors
COLOR_TEMP, COLOR_TEMP_WARNING, COLOR_RUNTIME
COLOR_WARNING, COLOR_ERROR, COLOR_SUCCESS

// UI Elements
COLOR_OVERLAY_BG, COLOR_OVERLAY_BORDER
COLOR_MENU_SELECT, COLOR_MENU_EDIT
```

Easy to modify for different color themes!

### 🔧 Technical Implementation

**Files Modified:**
- `src/main.cpp` - Main logic, screen management, job timer
- `src/display_ui.cpp` - All UI drawing functions
- `src/display_ui.h` - Function declarations, constants, color themes

**Key Functions Added:**
- `startJobTimer()` - Initialize job timer
- `pauseJobTimer()` - Pause during Power Pause
- `resumeJobTimer()` - Resume after Power Pause
- `getJobTimeSeconds()` - Get current job time
- `enterSupportScreen()` - Navigate to Support page
- `enterAboutScreen()` - Navigate to About page
- `drawRuntimeJobTime()` - Display job time
- `drawRuntimeOverTempOverlay()` - New overtemp UI
- `drawSupportScreen()` - Support page UI
- `drawAboutScreen()` - About page UI

**Functions Removed/Deprecated:**
- `drawRuntimePauseCountdown()` - No longer needed
- `drawRuntimeFooter()` - Redesigned UI
- `drawRuntimeMotorPower()` - Removed from display
- Old PID settings screen functions

### 📝 Notes for Future Updates

1. **Support Page Text**: Update placeholder text in `drawSupportScreen()` in `display_ui.cpp`
2. **Color Theme**: Modify color definitions in `display_ui.h` header
3. **Settings Expansion**: Room for 2+ more settings options in the Settings menu
4. **Power Pause Timeout**: Configurable via Settings (default 15 minutes = 900 seconds)

### ⚠️ Breaking Changes

- Settings menu no longer shows PID tuning parameters (Kp, Ki, Kd, Idle Dev, Start PSI)
- Old menu option "Firmware version" replaced with "About" page
- "Power pause" menu option integrated into "Settings"
- Runtime UI completely redesigned - old layout removed

### 🎯 User Experience Improvements

1. **Clearer Visual Feedback**: Color-coded Target PSI immediately shows status
2. **Better Space Utilization**: Larger fonts, optimized screen real estate
3. **Consistent Theming**: Unified color scheme across all screens
4. **Job Time Visibility**: Always visible during operation
5. **Safer Operation**: Must manually increase from 0 PSI, preventing accidental high-pressure starts
6. **Auto-timeout Safety**: Power Pause exits to menu after 15 minutes

## Testing Checklist

- [ ] Splash screen → Runtime screen transition
- [ ] Motor starts at 0 PSI
- [ ] Job timer starts/pauses/resumes correctly
- [ ] Color indicator changes (green/red) based on PSI accuracy
- [ ] Power Pause overlay shows countdown
- [ ] Power Pause timeout exits to menu after 15 min
- [ ] Overtemp overlay displays correctly
- [ ] Menu navigation (all 4 options)
- [ ] Settings menu (3 options + exit)
- [ ] Support page displays and returns to menu
- [ ] About page shows firmware version and runtime
- [ ] Temperature display in Fahrenheit
- [ ] Job time displays as HH:MM:SS

## Build Instructions

```bash
cd /home/astrid/dev/v2devapollo
pio run
```

## Flash Instructions

```bash
pio run --target upload
```
