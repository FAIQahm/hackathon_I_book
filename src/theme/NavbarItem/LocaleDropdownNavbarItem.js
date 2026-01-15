/**
 * Custom Locale Dropdown with localStorage persistence (T-140, T-142)
 *
 * Features:
 * - Saves language preference to localStorage
 * - Restores preference on page load
 * - Provides visual toggle in navbar
 */

import React, { useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAlternatePageUtils } from '@docusaurus/theme-common/internal';
import { translate } from '@docusaurus/Translate';
import DropdownNavbarItem from '@theme/NavbarItem/DropdownNavbarItem';

const LOCALE_STORAGE_KEY = 'preferred_locale';

// Get the stored locale preference
function getStoredLocale() {
  if (typeof window === 'undefined') return null;
  try {
    return localStorage.getItem(LOCALE_STORAGE_KEY);
  } catch {
    return null;
  }
}

// Store the locale preference
function setStoredLocale(locale) {
  if (typeof window === 'undefined') return;
  try {
    localStorage.setItem(LOCALE_STORAGE_KEY, locale);
  } catch {
    // localStorage might be disabled
  }
}

export default function LocaleDropdownNavbarItem({
  mobile,
  dropdownItemsBefore = [],
  dropdownItemsAfter = [],
  queryString = '',
  ...props
}) {
  const {
    i18n: { currentLocale, locales, localeConfigs },
  } = useDocusaurusContext();
  const alternatePageUtils = useAlternatePageUtils();
  const { search, hash } = useLocation();

  // Build locale items for dropdown
  const localeItems = locales.map((locale) => {
    const baseTo = `pathname://${alternatePageUtils.createUrl({
      locale,
      fullyQualified: false,
    })}`;
    const to = `${baseTo}${search}${hash}${queryString}`;

    return {
      label: localeConfigs[locale].label,
      lang: localeConfigs[locale].htmlLang,
      to,
      target: '_self',
      autoAddBaseUrl: false,
      className: locale === currentLocale ? 'dropdown__link--active' : '',
      style: localeConfigs[locale].direction === 'rtl' ? {
        fontFamily: "'Noto Nastaliq Urdu', serif",
        fontSize: '1.1rem',
      } : {},
      // Custom onClick to save preference
      onClick: () => {
        setStoredLocale(locale);
      },
    };
  });

  const items = [...dropdownItemsBefore, ...localeItems, ...dropdownItemsAfter];

  // Get the current locale label for the dropdown button
  const dropdownLabel = mobile
    ? translate({
        message: 'Languages',
        id: 'theme.navbar.mobileLanguageDropdown.label',
        description: 'The label for the mobile language switcher dropdown',
      })
    : localeConfigs[currentLocale].label;

  // Redirect to preferred locale on initial load (T-142)
  useEffect(() => {
    const storedLocale = getStoredLocale();

    // Only redirect if:
    // 1. We have a stored preference
    // 2. It's different from current locale
    // 3. User hasn't explicitly navigated to a locale URL
    if (
      storedLocale &&
      storedLocale !== currentLocale &&
      locales.includes(storedLocale)
    ) {
      // Check if user explicitly chose current locale (has locale in URL)
      const path = window.location.pathname;
      const hasExplicitLocale = locales.some(
        (l) => path.startsWith(`/${l}/`) || path === `/${l}`
      );

      // Only auto-redirect on root or if locale not explicitly set
      if (!hasExplicitLocale || path === '/') {
        const newUrl = alternatePageUtils.createUrl({
          locale: storedLocale,
          fullyQualified: false,
        });
        // Use replace to not add to history
        window.location.replace(newUrl);
      }
    } else if (!storedLocale) {
      // If no preference stored, save the current locale
      setStoredLocale(currentLocale);
    }
  }, []); // Only run once on mount

  return (
    <DropdownNavbarItem
      {...props}
      mobile={mobile}
      label={
        <span style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <span aria-hidden="true" style={{ fontSize: '1.2em' }}>
            {currentLocale === 'ur' ? '🇵🇰' : '🌐'}
          </span>
          {dropdownLabel}
        </span>
      }
      items={items}
    />
  );
}
