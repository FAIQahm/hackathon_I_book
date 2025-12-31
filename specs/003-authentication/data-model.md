# Data Model: User Authentication & Personalization

**Feature Branch**: `003-authentication`
**Created**: 2025-12-31
**Status**: Approved

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐
│      User       │───────│ UserPreference  │       │ PasswordReset   │
│                 │ 1   N │                 │       │ Token           │
└────────┬────────┘       └─────────────────┘       └────────┬────────┘
         │                                                    │
         │ 1                                                  │ 1
         │                                                    │
         ▼ N                                                  ▼
┌─────────────────┐                                 ┌─────────────────┐
│    Session      │                                 │      User       │
│                 │                                 │   (reference)   │
└─────────────────┘                                 └─────────────────┘
```

## Entities

### 1. User

A registered account holder in the system.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| email | String | Unique, Not Null, Max 255 | User's email address |
| password_hash | String | Not Null, Max 255 | bcrypt hashed password |
| created_at | Timestamp | Not Null, Default NOW | Account creation time |
| last_login | Timestamp | Nullable | Last successful login |
| account_status | Enum | Not Null, Default 'active' | active, locked, suspended |
| failed_attempts | Integer | Not Null, Default 0 | Consecutive failed logins |
| locked_until | Timestamp | Nullable | Account lockout expiry |
| onboarding_complete | Boolean | Not Null, Default false | Completed all questions |

**Constraints**:
- Email must be valid format
- Password hash is bcrypt with cost factor 12
- Account locks after 5 failed attempts for 15 minutes

**Indexes**:
- `idx_user_email` on (email) - Unique
- `idx_user_status` on (account_status)

---

### 2. UserPreference

Stored answers to onboarding questions.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| user_id | UUID | Foreign Key, Not Null | Reference to user |
| preference_key | String | Not Null, Max 50 | Preference dimension name |
| preference_value | String | Not Null, Max 255 | User's selected value |
| created_at | Timestamp | Not Null, Default NOW | Initial creation time |
| updated_at | Timestamp | Not Null, Default NOW | Last update time |

**Constraints**:
- Unique constraint on (user_id, preference_key)
- Valid preference_key values defined in PreferenceKey enum

**Indexes**:
- `idx_preference_user` on (user_id)
- `idx_preference_key` on (preference_key)
- `idx_preference_user_key` on (user_id, preference_key) - Unique

---

### 3. Session

An active authentication session.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Session identifier |
| user_id | UUID | Foreign Key, Not Null | Reference to user |
| token_hash | String | Not Null, Max 255 | Hashed JWT token |
| created_at | Timestamp | Not Null, Default NOW | Session start time |
| expires_at | Timestamp | Not Null | Session expiry time |
| device_info | String | Nullable, Max 500 | User agent / device |
| ip_address | String | Nullable, Max 45 | Client IP (IPv4/IPv6) |
| is_active | Boolean | Not Null, Default true | Session validity |

**Constraints**:
- Sessions expire after 24 hours by default
- Multiple sessions per user allowed

**Indexes**:
- `idx_session_user` on (user_id)
- `idx_session_token` on (token_hash)
- `idx_session_expires` on (expires_at)

---

### 4. PasswordResetToken

A temporary token for password recovery.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| user_id | UUID | Foreign Key, Not Null | Reference to user |
| token_hash | String | Not Null, Max 255 | Hashed reset token |
| created_at | Timestamp | Not Null, Default NOW | Token creation time |
| expires_at | Timestamp | Not Null | Token expiry (24 hours) |
| used_at | Timestamp | Nullable | When token was used |
| is_used | Boolean | Not Null, Default false | Token consumption status |

**Constraints**:
- Tokens expire after 24 hours (per FR-046)
- Single-use only
- New request invalidates previous tokens

**Indexes**:
- `idx_reset_user` on (user_id)
- `idx_reset_token` on (token_hash)
- `idx_reset_expires` on (expires_at)

---

### 5. OnboardingProgress

Partial progress through onboarding wizard (optional, for abandonment handling).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| user_id | UUID | Foreign Key, Not Null | Reference to user |
| current_step | Integer | Not Null, Default 1 | Current wizard step (1-3) |
| answers_json | JSONB | Nullable | Partial answers as JSON |
| last_updated | Timestamp | Not Null, Default NOW | Last activity time |

**Constraints**:
- One progress record per user
- Deleted upon onboarding completion

---

## Enumerations

### AccountStatus
```python
class AccountStatus(str, Enum):
    ACTIVE = "active"
    LOCKED = "locked"
    SUSPENDED = "suspended"
```

### PreferenceKey
```python
class PreferenceKey(str, Enum):
    TECHNICAL_BACKGROUND = "technical_background"
    DOMAIN_KNOWLEDGE = "domain_knowledge"
    LEARNING_GOAL = "learning_goal"
    PREFERRED_DEPTH = "preferred_depth"
    CODE_EXAMPLES = "code_examples"
    TIME_COMMITMENT = "time_commitment"
    FOCUS_AREA = "focus_area"
    LANGUAGE_PREFERENCE = "language_preference"
    PRIOR_AI_EXPERIENCE = "prior_ai_experience"
    NOTIFICATION_PREFERENCE = "notification_preference"
```

### PreferenceValues (by key)
```python
PREFERENCE_OPTIONS = {
    "technical_background": ["beginner", "intermediate", "advanced"],
    "domain_knowledge": ["none", "some", "experienced"],
    "learning_goal": ["career_change", "skill_enhancement", "academic_research", "hobby"],
    "preferred_depth": ["overview", "balanced", "deep_dive"],
    "code_examples": ["not_important", "somewhat", "very_important"],
    "time_commitment": ["under_2_hours", "2_to_5_hours", "over_5_hours"],
    "focus_area": ["ros2", "simulation", "computer_vision", "machine_learning"],
    "language_preference": ["english", "urdu"],
    "prior_ai_experience": ["yes", "no", "learning"],
    "notification_preference": ["yes", "no"]
}
```

---

## Relationships

| Parent | Child | Cardinality | Description |
|--------|-------|-------------|-------------|
| User | UserPreference | 1:N | User has many preferences |
| User | Session | 1:N | User can have multiple sessions |
| User | PasswordResetToken | 1:N | User can request multiple resets |
| User | OnboardingProgress | 1:1 | User has one progress record |

---

## Database Schema (Neon Postgres)

```sql
-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_login TIMESTAMP WITH TIME ZONE,
    account_status VARCHAR(20) DEFAULT 'active',
    failed_attempts INTEGER DEFAULT 0,
    locked_until TIMESTAMP WITH TIME ZONE,
    onboarding_complete BOOLEAN DEFAULT FALSE
);

CREATE UNIQUE INDEX idx_user_email ON users(email);
CREATE INDEX idx_user_status ON users(account_status);

-- User preferences table
CREATE TABLE user_preferences (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    preference_key VARCHAR(50) NOT NULL,
    preference_value VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    UNIQUE(user_id, preference_key)
);

CREATE INDEX idx_preference_user ON user_preferences(user_id);
CREATE INDEX idx_preference_key ON user_preferences(preference_key);

-- Sessions table
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    device_info VARCHAR(500),
    ip_address VARCHAR(45),
    is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX idx_session_user ON sessions(user_id);
CREATE INDEX idx_session_token ON sessions(token_hash);
CREATE INDEX idx_session_expires ON sessions(expires_at);

-- Password reset tokens table
CREATE TABLE password_reset_tokens (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    used_at TIMESTAMP WITH TIME ZONE,
    is_used BOOLEAN DEFAULT FALSE
);

CREATE INDEX idx_reset_user ON password_reset_tokens(user_id);
CREATE INDEX idx_reset_token ON password_reset_tokens(token_hash);
CREATE INDEX idx_reset_expires ON password_reset_tokens(expires_at);

-- Onboarding progress table
CREATE TABLE onboarding_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    current_step INTEGER DEFAULT 1,
    answers_json JSONB,
    last_updated TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_onboarding_user ON onboarding_progress(user_id);

-- Trigger to update updated_at on preferences
CREATE OR REPLACE FUNCTION update_updated_at()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER preference_updated_at
    BEFORE UPDATE ON user_preferences
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at();
```

---

## Pydantic Models (Python)

```python
from pydantic import BaseModel, Field, EmailStr
from typing import Optional, List, Dict
from datetime import datetime
from uuid import UUID
from enum import Enum

class AccountStatus(str, Enum):
    ACTIVE = "active"
    LOCKED = "locked"
    SUSPENDED = "suspended"

class PreferenceKey(str, Enum):
    TECHNICAL_BACKGROUND = "technical_background"
    DOMAIN_KNOWLEDGE = "domain_knowledge"
    LEARNING_GOAL = "learning_goal"
    PREFERRED_DEPTH = "preferred_depth"
    CODE_EXAMPLES = "code_examples"
    TIME_COMMITMENT = "time_commitment"
    FOCUS_AREA = "focus_area"
    LANGUAGE_PREFERENCE = "language_preference"
    PRIOR_AI_EXPERIENCE = "prior_ai_experience"
    NOTIFICATION_PREFERENCE = "notification_preference"

# Request models
class RegisterRequest(BaseModel):
    email: EmailStr
    password: str = Field(min_length=8)

class LoginRequest(BaseModel):
    email: EmailStr
    password: str

class PasswordResetRequest(BaseModel):
    email: EmailStr

class PasswordResetConfirm(BaseModel):
    token: str
    new_password: str = Field(min_length=8)

class PreferenceUpdate(BaseModel):
    preference_key: PreferenceKey
    preference_value: str

class OnboardingSubmission(BaseModel):
    preferences: Dict[PreferenceKey, str]

# Response models
class UserResponse(BaseModel):
    id: UUID
    email: str
    created_at: datetime
    account_status: AccountStatus
    onboarding_complete: bool

class PreferenceResponse(BaseModel):
    preference_key: PreferenceKey
    preference_value: str
    updated_at: datetime

class UserPreferencesResponse(BaseModel):
    user_id: UUID
    preferences: List[PreferenceResponse]
    onboarding_complete: bool

class AuthResponse(BaseModel):
    user: UserResponse
    access_token: str
    token_type: str = "bearer"
    expires_in: int

class MessageResponse(BaseModel):
    message: str
    success: bool = True
```

---

## TypeScript Interfaces (Frontend)

```typescript
// Enums
type AccountStatus = 'active' | 'locked' | 'suspended';

type PreferenceKey =
  | 'technical_background'
  | 'domain_knowledge'
  | 'learning_goal'
  | 'preferred_depth'
  | 'code_examples'
  | 'time_commitment'
  | 'focus_area'
  | 'language_preference'
  | 'prior_ai_experience'
  | 'notification_preference';

// Request types
interface RegisterRequest {
  email: string;
  password: string;
}

interface LoginRequest {
  email: string;
  password: string;
}

interface OnboardingSubmission {
  preferences: Record<PreferenceKey, string>;
}

// Response types
interface User {
  id: string;
  email: string;
  createdAt: Date;
  accountStatus: AccountStatus;
  onboardingComplete: boolean;
}

interface Preference {
  preferenceKey: PreferenceKey;
  preferenceValue: string;
  updatedAt: Date;
}

interface UserPreferences {
  userId: string;
  preferences: Preference[];
  onboardingComplete: boolean;
}

interface AuthResponse {
  user: User;
  accessToken: string;
  tokenType: string;
  expiresIn: number;
}

// Auth state
interface AuthState {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
}

// Onboarding state
interface OnboardingState {
  currentStep: number;
  answers: Partial<Record<PreferenceKey, string>>;
  isComplete: boolean;
}
```

---

## Preference Options Reference

| Key | Display Question | Options |
|-----|------------------|---------|
| technical_background | What is your programming experience level? | Beginner, Intermediate, Advanced |
| domain_knowledge | How familiar are you with robotics concepts? | None, Some, Experienced |
| learning_goal | What do you want to achieve? | Career change, Skill enhancement, Academic research, Hobby |
| preferred_depth | Do you prefer overviews or details? | Overview, Balanced, Deep-dive |
| code_examples | How important are runnable code examples? | Not important, Somewhat, Very important |
| time_commitment | How much time can you dedicate weekly? | <2 hours, 2-5 hours, 5+ hours |
| focus_area | Which topic interests you most? | ROS 2, Simulation, Computer Vision, Machine Learning |
| language_preference | Preferred content language? | English, Urdu |
| prior_ai_experience | Have you worked with AI/ML before? | Yes, No, Learning |
| notification_preference | Would you like email updates? | Yes, No |
