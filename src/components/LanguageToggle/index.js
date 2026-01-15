/**
 * LanguageToggle Component (T-140)
 *
 * A simple toggle button for switching between English and Urdu.
 * Saves preference to localStorage (T-142).
 */

import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAlternatePageUtils } from '@docusaurus/theme-common/internal';
import styles from './styles.module.css';

const LOCALE_STORAGE_KEY = 'preferred_locale';

function setStoredLocale(locale) {
  if (typeof window === 'undefined') return;
  try {
    localStorage.setItem(LOCALE_STORAGE_KEY, locale);
  } catch {
    // localStorage might be disabled
  }
}

export default function LanguageToggle({ className }) {
  const {
    i18n: { currentLocale, locales, localeConfigs },
  } = useDocusaurusContext();
  const alternatePageUtils = useAlternatePageUtils();
  const { search, hash } = useLocation();

  // Find the "other" locale
  const otherLocale = currentLocale === 'en' ? 'ur' : 'en';
  const otherConfig = localeConfigs[otherLocale];

  // Build the URL for the other locale
  const otherUrl = `pathname://${alternatePageUtils.createUrl({
    locale: otherLocale,
    fullyQualified: false,
  })}${search}${hash}`;

  const handleSwitch = () => {
    setStoredLocale(otherLocale);
    window.location.href = otherUrl;
  };

  const isUrdu = currentLocale === 'ur';

  return (
    <button
      className={`${styles.toggle} ${className || ''}`}
      onClick={handleSwitch}
      aria-label={`Switch to ${otherConfig.label}`}
      title={`Switch to ${otherConfig.label}`}
    >
      <span className={styles.toggleTrack}>
        <span
          className={`${styles.toggleThumb} ${isUrdu ? styles.toggleThumbUrdu : ''}`}
        >
          {isUrdu ? 'UR' : 'EN'}
        </span>
        <span className={styles.toggleLabel}>
          {isUrdu ? 'English' : 'اردو'}
        </span>
      </span>
    </button>
  );
}
