// screen_capture.m
#import <Foundation/Foundation.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>            // SCShareableContent, SCContentFilter, SCStreamConfiguration, SCScreenshotManager
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h> // UTTypePNG.identifier
#import <CoreVideo/CoreVideo.h>                          // kCVPixelFormatType_32BGRA
#import <ImageIO/ImageIO.h>                              // CGImageDestination
#import <CoreGraphics/CoreGraphics.h>  // for CGMainDisplayID, CGDisplayPixelsWide/High, CGImageRelease, etc.


int main(int argc, const char * argv[]) {
    @autoreleasepool {
        // Семафор, чтобы дождаться асинхронного колбэка
        dispatch_semaphore_t sem = dispatch_semaphore_create(0);

        // 1. Получаем доступный контент (дисплеи, окна)
        [SCShareableContent getShareableContentWithCompletionHandler:^(SCShareableContent * _Nullable content, NSError * _Nullable error) {
				if (error || content.displays.count == 0) {
					NSLog(@"Ошибка получения контента: %@", error);
					dispatch_semaphore_signal(sem);
					return;
				}

				// 2. Фильтр по главному дисплею
				SCDisplay *display = content.displays[0];
				SCContentFilter *filter = [[SCContentFilter alloc] initWithDisplay:display
															 excludingApplications:@[]
																  exceptingWindows:@[]];

				// 3. Конфигурация скриншота
				SCStreamConfiguration *cfg = [[SCStreamConfiguration alloc] init];
				cfg.width       = CGDisplayPixelsWide(CGMainDisplayID());
				cfg.height      = CGDisplayPixelsHigh(CGMainDisplayID());
				cfg.pixelFormat = kCVPixelFormatType_32BGRA;
				cfg.showsCursor = YES;

				// 4. Захват одного кадра
				[SCScreenshotManager
					captureImageWithFilter:filter
							 configuration:cfg
						 completionHandler:^(CGImageRef  _Nullable image, NSError * _Nullable err) {
						if (image && !err) {
							// 5. Сохранение в ~/Desktop/screenshot.png
							NSString *outPath = [NSHomeDirectory()
													stringByAppendingPathComponent:@"Desktop/screenshot.png"];
							CFURLRef url = CFURLCreateWithFileSystemPath(
								kCFAllocatorDefault,
								(__bridge CFStringRef)outPath,
								kCFURLPOSIXPathStyle,
								false
								);
							CGImageDestinationRef dest = CGImageDestinationCreateWithURL(
								url,
								(__bridge CFStringRef)UTTypePNG.identifier,
								1,
								NULL
								);
							if (dest) {
								CGImageDestinationAddImage(dest, image, NULL);
								CGImageDestinationFinalize(dest);
								CFRelease(dest);
								NSLog(@"✅ Saved screenshot to %@", outPath);
							}
							CFRelease(url);
							CGImageRelease(image);
						} else {
							NSLog(@"❌ Capture failed: %@", err);
						}
						dispatch_semaphore_signal(sem);
					}];
			}];

        // Ждём завершения захвата
        dispatch_semaphore_wait(sem, DISPATCH_TIME_FOREVER);
    }
    return 0;
}
