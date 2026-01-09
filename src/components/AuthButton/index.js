import React, { useState } from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import AuthModal from '@site/src/components/AuthModal';
import styles from './styles.module.css';

export default function AuthButton() {
  const [showModal, setShowModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);
  const { user, logout, loading } = useAuth();

  if (loading) {
    return <div className={styles.loading}>...</div>;
  }

  if (user) {
    return (
      <div className={styles.userMenu}>
        <button
          className={styles.userButton}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label="User menu"
        >
          <span className={styles.avatar}>
            {user.email?.charAt(0).toUpperCase() || 'U'}
          </span>
          <span className={styles.email}>{user.email}</span>
        </button>

        {showDropdown && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <span className={styles.role}>{user.role}</span>
              <span className={styles.userEmail}>{user.email}</span>
            </div>
            <hr className={styles.divider} />
            <button
              className={styles.logoutBtn}
              onClick={() => {
                logout();
                setShowDropdown(false);
              }}
            >
              Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <button
        className={styles.signInBtn}
        onClick={() => setShowModal(true)}
      >
        Sign In
      </button>

      <AuthModal
        isOpen={showModal}
        onClose={() => setShowModal(false)}
      />
    </>
  );
}
