#ifndef	_FUSES_H
	#ifdef _INCLUDE_FUSES
		#if defined (__AVR_ATmega168__)
		FUSES = {
			.low = FUSE_CKSEL3 & FUSE_SUT0,
			.high = FUSE_SPIEN,
			.extended = FUSE_BOOTSZ1 & FUSE_BOOTSZ0,
		};
		#elif defined (__AVR_ATmega328P__)
		FUSES = {
			.low = FUSE_CKSEL3 & FUSE_SUT0,
			.high = FUSE_SPIEN & FUSE_BOOTSZ0 & FUSE_BOOTSZ1,
			.extended = EFUSE_DEFAULT,
		};
		#elif defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__)
		FUSES = {
			.low = FUSE_CKSEL3 & FUSE_SUT0,
			.high = FUSE_SPIEN & FUSE_BOOTSZ0 & FUSE_BOOTSZ1,
			.extended = EFUSE_DEFAULT,
		};
		#elif defined (__AVR_ATmega1280__)
		FUSES = {
			.low = FUSE_CKSEL3 & FUSE_SUT0,
			.high = FUSE_SPIEN & FUSE_BOOTSZ0 & FUSE_BOOTSZ1,
			.extended = EFUSE_DEFAULT,
		};
		#else
			#warning No fuse definitions for this chip in fuses.h!
		#endif
	#endif	/* _INCLUDE_FUSES */
#endif	/* _FUSES_H */
